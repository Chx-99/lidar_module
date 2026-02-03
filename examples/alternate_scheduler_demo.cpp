/**
 * @file alternate_scheduler_demo.cpp
 * @brief 交替采集调度器演示程序
 * 
 * 这个独立示例展示了如何实现多设备的时分复用调度系统
 * 可用于雷达、传感器等需要避免相互干扰的设备
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <functional>
#include <atomic>
#include <iomanip>

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
        
        std::cout << "[调度器] 启动 - 相位=" << phase_index_ 
                  << "/" << phase_count_ 
                  << ", 周期=" << period_ms_ << "ms\n";
        
        // 启动调度线程
        scheduler_thread_ = std::thread([this]() {
            while (enabled_) {
                tick();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
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
        if (switch_callback_) {
            switch_callback_(want_on);
            device_on_ = want_on;
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
};

// ============================================================================
// 辅助函数：格式化时间显示
// ============================================================================

std::string formatTime() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ) % 1000;
    
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&timer);
    
    std::ostringstream oss;
    oss << std::put_time(&bt, "%H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

// ============================================================================
// 模拟设备类
// ============================================================================

class MockDevice {
public:
    MockDevice(const std::string& name, int phase_index, int phase_count,
               int on_ms, int guard_ms, int64_t epoch_ns)
        : name_(name)
        , scheduler_(phase_count, phase_index, on_ms, guard_ms, epoch_ns)
    {
    }

    void start() {
        scheduler_.start([this](bool on) {
            std::cout << "[" << formatTime() << "] " 
                      << name_ << " -> " 
                      << (on ? "\033[32m开启\033[0m" : "\033[31m关闭\033[0m") 
                      << std::endl;
        });
    }

    void stop() {
        scheduler_.stop();
    }

private:
    std::string name_;
    AlternateScheduler scheduler_;
};

// ============================================================================
// 主函数：演示案例
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "交替采集调度器演示\n";
    std::cout << "========================================\n\n";

    // ===== 配置参数 =====
    const int DEVICE_COUNT = 3;      // 3个设备
    const int ON_MS = 2000;          // 工作2秒
    const int GUARD_MS = 500;        // 保护0.5秒
    
    // 统一的时间基准点（所有设备必须使用同一个）
    const int64_t epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();

    std::cout << "配置参数:\n";
    std::cout << "  设备数量: " << DEVICE_COUNT << "\n";
    std::cout << "  工作时间: " << ON_MS << " ms\n";
    std::cout << "  保护时间: " << GUARD_MS << " ms\n";
    std::cout << "  总周期: " << DEVICE_COUNT * (ON_MS + GUARD_MS) << " ms\n";
    std::cout << "\n时间分配:\n";
    
    for (int i = 0; i < DEVICE_COUNT; ++i) {
        int start = i * (ON_MS + GUARD_MS);
        int work_end = start + ON_MS;
        int phase_end = start + ON_MS + GUARD_MS;
        std::cout << "  设备" << i << ": [" << start << "ms - " << work_end 
                  << "ms] 工作, [" << work_end << "ms - " << phase_end 
                  << "ms] 保护\n";
    }
    std::cout << "\n开始运行...\n";
    std::cout << "----------------------------------------\n";

    // ===== 创建并启动设备 =====
    std::vector<std::unique_ptr<MockDevice>> devices;
    for (int i = 0; i < DEVICE_COUNT; ++i) {
        auto device = std::make_unique<MockDevice>(
            "设备" + std::to_string(i),
            i,              // phase_index
            DEVICE_COUNT,   // phase_count
            ON_MS,
            GUARD_MS,
            epoch_ns
        );
        device->start();
        devices.push_back(std::move(device));
    }

    // ===== 运行指定时间 =====
    const int RUN_SECONDS = 15;  // 运行15秒（约2个完整周期）
    std::cout << "将运行 " << RUN_SECONDS << " 秒...\n";
    std::cout << "（观察设备按序开关，相互不重叠）\n";
    std::cout << "----------------------------------------\n\n";

    std::this_thread::sleep_for(std::chrono::seconds(RUN_SECONDS));

    // ===== 停止所有设备 =====
    std::cout << "\n----------------------------------------\n";
    std::cout << "停止所有设备...\n";
    for (auto& device : devices) {
        device->stop();
    }

    std::cout << "\n演示完成！\n";
    std::cout << "========================================\n";

    return 0;
}
