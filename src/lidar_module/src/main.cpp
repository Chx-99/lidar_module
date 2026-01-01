#include "base/data_struct.h"
#include "base/data_analysis.h"
#include "concurrentqueue.h"
#include "lidar/lidar_base.h"
#include "lidar/lidar_scan.h"
#include "network/port_scan.h"
#include "process/process_work.h"
#include "queue/data_queue.h"


#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

using namespace lidar_base;
using namespace lidar_scanner;
using namespace base_frame;
using namespace frame_tools;
using namespace free_queue;
using namespace process_worker;

int main(int /* argc */, char** /* argv */) {
    boost::asio::io_context io_context;

    // 创建work_guard保持io_context运行（全局管理）
    auto work_guard = boost::asio::make_work_guard(io_context);

    
    LidarScanner scanner(5);
    auto lidars = scanner.searchLidar();

    // 使用智能指针管理 Lidar 对象（Lidar 包含不可复制的成员如 socket、mutex）
    std::vector<std::unique_ptr<lidar_base::Lidar>> lidar_devices;

    // 遍历所有雷达
    for (const auto& [sn, info] : lidars) {
        auto lidar = std::make_unique<Lidar>(io_context, info.lidar_ip, info.local_ip, sn);
        lidar_devices.push_back(std::move(lidar));
    }

    // 根据雷达数量动态计算线程数
    // 策略：每个雷达分配一定的线程资源，但设置上下限
    // - 1个雷达：1个线程
    // - 2-10个雷达：2个线程
    // - 11-20个雷达：4个线程
    // - 21个以上：6个线程
    size_t lidar_count = lidar_devices.size();
    size_t thread_count = 1;  // 最小1个线程

    if (lidar_count == 0) {
        std::cerr << "未发现任何雷达，退出程序" << std::endl;
        return 1;
    } else if (lidar_count <= 10) {
        thread_count = std::min<size_t>(2, lidar_count);  // 1-10个雷达：最多2个线程
    } else if (lidar_count <= 20) {
        thread_count = 4;  // 11-20个雷达：4个线程
    } else {
        thread_count = 6;  // 21个以上雷达：6个线程（最大值）
    }

    std::cout << "检测到 " << lidar_count << " 个雷达，分配 " << thread_count << " 个IO线程" << std::endl;

    // 4. 启动线程池（在雷达创建后）
    std::vector<std::thread> io_threads;
    for (size_t i = 0; i < thread_count; ++i) {
        io_threads.emplace_back([&io_context, i]() {
            // pthread_setname_np(pthread_self(),
            //     ("IO-" + std::to_string(i)).c_str());
            io_context.run();
        });
    }

    // 等待io_context启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 连接所有雷达
    bool all_connected = true;
    for (auto& lidar : lidar_devices) {
        if (!lidar->connect()) {
            std::cerr << "雷达连接失败" << std::endl;
            all_connected = false;
            // 继续尝试连接其他雷达，或者选择break退出
        } else {
            std::cout << "雷达连接成功！" << std::endl;
        }
    }
    
    if (!all_connected) {
        std::cerr << "部分或全部雷达连接失败" << std::endl;
        // 可以选择继续运行已连接的雷达，或者退出
        // io_context.stop();
        // for (auto& t : io_threads) { t.join(); }
        // return 1;
    }

    // 创建线程池处理器（根据雷达数量动态计算处理线程数）
    // 策略：每5-10台雷达分配1个处理线程，上限8个线程
    size_t processor_thread_count = std::max<size_t>(1, std::min<size_t>(8, (lidar_count + 9) / 10));
    std::cout << "创建 " << processor_thread_count << " 个数据处理线程" << std::endl;
    
    ProcessorThreadPool processor;

    // 设置点云处理回调（优化内存管理）
    processor.setPointCloudBatchCallback(pointcloud_process);

    // 设置IMU处理回调
    processor.setIMUCallback(imu_process);


    // 启动线程池
    processor.start();





    // 等待用户中断
    std::cout << "按Enter键退出..." << std::endl;
    std::cin.get();

    // 停止线程池
    processor.stop();

    // 断开所有雷达连接
    for (auto& lidar : lidar_devices) {
        lidar->disconnect(); 
    }

    // 释放work_guard，允许io_context退出
    work_guard.reset();
    
    // 等待所有IO线程
    for (auto& t : io_threads) { t.join(); }
    
    return 0;
}