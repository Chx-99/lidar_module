
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <functional>
#include <atomic>
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
        
        // 启动调度线程
        scheduler_thread_ = std::thread([this]() {
            RCLCPP_DEBUG(logger_, "调度器线程已启动，开始循环");
            int tick_count = 0;
            while (enabled_) {
                tick();
                tick_count++;
                if (tick_count % 500 == 0) {  // 每5秒打印一次（500 * 10ms）
                    RCLCPP_DEBUG(logger_, "调度器运行中，已执行 %d 次tick", tick_count);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            RCLCPP_DEBUG(logger_, "调度器线程已停止");
        });
    }

    /**
     * @brief 停止调度器
     */
    void stop() {
        enabled_ = false;
        if (scheduler_thread_.joinable()) {
            scheduler_thread_.join();
        }
        // 确保设备关闭
        if (switch_callback_ && device_on_) {
            switch_callback_(false);
            device_on_ = false;
        }
    }

    /**
     * @brief 获取当前设备状态
     */
    bool isDeviceOn() const {
        return device_on_;
    }

private:
    /**
     * @brief 定时器触发函数，每10ms执行一次
     */
    void tick() {
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
     * 
     * 核心算法：
     * 1. 计算从时间基准开始经过的时间
     * 2. 对周期取模，得到当前在周期内的位置
     * 3. 判断是否在自己的相位内
     * 4. 在相位内再判断是否在工作时间内
     */
    bool inActiveWindow(int64_t now_ns) const {
        if (phase_count_ <= 0 || period_ms_ <= 0) {
            return false;
        }

        const int64_t period_ns = static_cast<int64_t>(period_ms_) * 1'000'000LL;
        const int64_t t_ns = now_ns - epoch_ns_;

        // 还没到时间基准点
        if (t_ns < 0) {
            return false;
        }

        // 计算在当前周期内的位置
        const int64_t mod = t_ns % period_ns;

        // 计算每个相位的时间长度
        const int64_t seg_ns = period_ns / phase_count_;
        
        // 计算本设备相位的起止时间
        const int64_t phase_start = static_cast<int64_t>(phase_index_) * seg_ns;
        const int64_t phase_end = phase_start + seg_ns;

        // 不在自己的相位内
        if (mod < phase_start || mod >= phase_end) {
            return false;
        }

        // 在相位内，判断是否在工作时间内
        const int64_t offset_in_phase = mod - phase_start;
        const int64_t on_ns = static_cast<int64_t>(on_ms_) * 1'000'000LL;

        // [0, on_ns) 为工作区，其余为保护区
        return offset_in_phase < on_ns;
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
    rclcpp::Logger logger_;     // ROS日志对象
};