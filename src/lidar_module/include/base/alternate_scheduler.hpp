
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief 交替采集调度器
 * 
 * 核心功能：
 * 1. 时间同步：所有设备使用统一的时间基准
 * 2. 相位分配：每个设备分配独立的时间片
 * 3. 周期调度：自动循环执行开关操作
 */
class AlternateScheduler {
public:
    /**
     * @param phase_count 相位总数（设备数量）
     * @param phase_index 当前设备的相位索引（0, 1, 2...）
     * @param on_ms 工作时间（毫秒）
     * @param guard_ms 保护时间（毫秒）
     * @param epoch_ns 时间基准点（纳秒）
     */
    AlternateScheduler(int phase_count, int phase_index, 
                       int on_ms, int guard_ms, 
                       int64_t epoch_ns)
        : phase_count_(phase_count)
        , phase_index_(phase_index)
        , on_ms_(on_ms)
        , guard_ms_(guard_ms)
        , epoch_ns_(epoch_ns)
        , enabled_(false)
        , device_on_(false)
        , logger_(rclcpp::get_logger("AlternateScheduler"))
    {
        // 计算总周期
        period_ms_ = phase_count_ * (on_ms_ + guard_ms_);
    }

    /**
     * @brief 启动调度器
     * @param switch_callback 设备开关回调函数 (true=开, false=关)
     */
    void start(std::function<void(bool)> switch_callback) {
        enabled_ = true;
        switch_callback_ = switch_callback;
        
        RCLCPP_INFO(logger_, "调度器启动 - 相位=%d/%d, 周期=%dms", 
                    phase_index_, phase_count_, period_ms_);
        
        // 启动调度线程（事件驱动，CPU占用接近0%）
        scheduler_thread_ = std::thread([this]() {
            RCLCPP_INFO(logger_, "调度器线程已启动");
            
            while (enabled_) {
                // 计算下次切换时间点（只在切换时计算一次）
                auto next_switch_time = calculateNextSwitchTime();
                
                // 精确等待，期间CPU=0%
                std::unique_lock<std::mutex> lock(cv_mutex_);
                if (cv_.wait_until(lock, next_switch_time, [this] { return !enabled_; })) {
                    break; // 收到停止信号
                }
                
                // 执行状态切换
                if (enabled_) {
                    switchDeviceAtTime();
                }
            }
            
            RCLCPP_INFO(logger_, "调度器线程已停止");
        });
    }

    /**
     * @brief 停止调度器
     */
    void stop() {
        RCLCPP_INFO(logger_, "正在停止调度器...");
        enabled_ = false;
        cv_.notify_one(); // 唤醒等待线程
        
        if (scheduler_thread_.joinable()) {
            scheduler_thread_.join();
        }
        
        // 确保设备关闭
        if (switch_callback_ && device_on_) {
            RCLCPP_INFO(logger_, "关闭设备");
            switch_callback_(false);
            device_on_ = false;
        }
        
        RCLCPP_INFO(logger_, "调度器已停止");
    }

    /**
     * @brief 获取当前设备状态
     */
    bool isDeviceOn() const {
        return device_on_;
    }

private:
    /**
     * @brief 计算下次状态切换的时间点
     */
    std::chrono::steady_clock::time_point calculateNextSwitchTime() const {
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

        const int64_t period_ns = static_cast<int64_t>(period_ms_) * 1'000'000LL;
        const int64_t elapsed = now_ns - epoch_ns_;

        // 还没到时间基准点
        if (elapsed < 0) {
            return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(epoch_ns_));
        }

        // 计算周期内位置和相位边界
        const int64_t offset = elapsed % period_ns;
        const int64_t seg_ns = period_ns / phase_count_;
        const int64_t phase_start = static_cast<int64_t>(phase_index_) * seg_ns;
        const int64_t on_ns = static_cast<int64_t>(on_ms_) * 1'000'000LL;
        const int64_t phase_on = phase_start;
        const int64_t phase_off = phase_start + on_ns;

        // 确定下次切换时间（开启或关闭）
        int64_t next_offset;
        if (offset < phase_on) {
            next_offset = phase_on;  // 等待开启
        } else if (offset < phase_off) {
            next_offset = phase_off; // 等待关闭
        } else {
            next_offset = period_ns + phase_on; // 下一周期的开启
        }

        const int64_t next_switch_ns = epoch_ns_ + (elapsed - offset + next_offset);
        return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(next_switch_ns));
    }

    /**
     * @brief 在切换时间点执行设备开关（避免重复时间获取和计算）
     */
    void switchDeviceAtTime() {
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

        const bool want_on = inActiveWindow(now_ns);
        
        // 状态变化时才切换
        if (want_on != device_on_) {
            switchDevice(want_on);
        }
    }

    /**
     * @brief 判断当前时刻是否应该开启设备
     */
    bool inActiveWindow(int64_t now_ns) const {
        const int64_t period_ns = static_cast<int64_t>(period_ms_) * 1'000'000LL;
        const int64_t elapsed = now_ns - epoch_ns_;

        if (elapsed < 0) return false;

        const int64_t offset = elapsed % period_ns;
        const int64_t seg_ns = period_ns / phase_count_;
        const int64_t phase_start = static_cast<int64_t>(phase_index_) * seg_ns;
        const int64_t on_ns = static_cast<int64_t>(on_ms_) * 1'000'000LL;

        // 在自己相位内且在工作时间内
        return (offset >= phase_start) && (offset < phase_start + on_ns);
    }

    /**
     * @brief 执行设备开关操作
     */
    void switchDevice(bool want_on) {
        RCLCPP_INFO(logger_, "切换设备状态: %s", want_on ? "ON" : "OFF");
        if (switch_callback_) {
            switch_callback_(want_on);
            device_on_ = want_on;
        } else {
            RCLCPP_WARN(logger_, "回调函数未设置!");
        }
    }

private:
    int phase_count_;           // 相位总数
    int phase_index_;           // 当前相位索引
    int on_ms_;                 // 工作时间
    int guard_ms_;              // 保护时间
    int period_ms_;             // 总周期
    int64_t epoch_ns_;          // 时间基准点

    std::atomic<bool> enabled_; // 调度器运行状态
    bool device_on_;            // 设备当前状态
    
    std::function<void(bool)> switch_callback_;
    std::thread scheduler_thread_;
    std::condition_variable cv_;    // 条件变量，用于精确等待
    std::mutex cv_mutex_;           // 条件变量的互斥锁
    rclcpp::Logger logger_;         // ROS日志对象
};