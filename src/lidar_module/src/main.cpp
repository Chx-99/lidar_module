#include "base/data_struct.h"
#include "concurrentqueue.h"
#include "lidar/lidar_base.h"
#include "lidar/lidar_scan.h"
#include "network/port_scan.h"
#include "queue/data_queue.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

using namespace std::chrono;

// 定义时间测量宏
#define MEASURE_TIME(operation, iterations, name)                                                               \
    auto start_##name = high_resolution_clock::now();                                                           \
    for (int i = 0; i < iterations; ++i) { operation; }                                                         \
    auto end_##name = high_resolution_clock::now();                                                             \
    auto duration_##name = duration_cast<nanoseconds>(end_##name - start_##name);                               \
    std::cout << #name << " (" << iterations << " iterations) 执行时间: " << duration_##name.count() << " 纳秒" \
              << std::endl;                                                                                     \
    std::cout << "平均每次执行时间: " << duration_##name.count() / iterations << " 纳秒" << std::endl;

using namespace lidar_module;
using namespace base_frame;
using namespace frame_tools;

int main(int /* argc */, char** /* argv */) {
    boost::asio::io_context io_context;

    // 创建work_guard保持io_context运行（全局管理）
    auto work_guard = boost::asio::make_work_guard(io_context);


    lidar_module::LidarScanner scanner(5);
    auto lidars = scanner.searchLidar();

    // 使用智能指针管理 Lidar 对象（Lidar 包含不可复制的成员如 socket、mutex）
    std::vector<std::unique_ptr<Lidar>> lidar_devices;

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
    try {
        for (auto& lidar : lidar_devices) {
            lidar->connect();  // 使用 -> 因为是指针
            std::cout << "雷达连接成功！" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "雷达连接失败: " << e.what() << std::endl;
        io_context.stop();
        for (auto& t : io_threads) { t.join(); }
        return 1;
    }

    // 启动消费者线程处理ACK队列
    std::atomic<bool> running{true};
    std::thread consumer_thread([&running]() {
        auto& ack_queue = free_queue::SharedQueues::instance().ackQueue();
        while (running.load()) {
            free_queue::AckPacket packet;
            if (ack_queue.try_dequeue(packet)) {
                auto ack_tab = getAckNameCompileTime(GET_ACK_SET(packet.data.data()), GET_ACK_ID(packet.data.data()));

                // 使用if-else替代switch，因为ack_tab是字符串
                if (ack_tab == "HandShake ACK") {
                    HandShakeACK handshake = fromVector<HandShakeACK>(packet.data);
                    std::cout << "收到 HandShake ACK: " << handshake << std::endl;
                } else if (ack_tab == "HeartBeat ACK") {
                    // HeartBeatACK heartbeat = fromVector<HeartBeatACK>(packet.data);
                    // std::cout << "收到 HeartBeat ACK: " << heartbeat << std::endl;
                } else if (ack_tab == "SetLaserStatus ACK") {
                    SetLaserStatusACK set_laser_status = fromVector<SetLaserStatusACK>(packet.data);
                    std::cout << "收到 SetLaserStatus ACK: " << set_laser_status << std::endl;
                } else if (ack_tab == "Set IMU Frequency ACK") {
                    SetIMUFrequencyACK set_imu_frequency = fromVector<SetIMUFrequencyACK>(packet.data);
                    std::cout << "收到 Set IMU Frequency ACK: " << set_imu_frequency << std::endl;
                } else {
                    std::cout << "收到未知 ACK/MSG" << std::endl;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    });

    // 消费点云线程
    std::thread pointcloud_consumer_thread([&running]() {
        auto& pointcloud_queue = free_queue::SharedQueues::instance().pointCloudQueue();
        while (running.load()) {
            free_queue::PointCloudPacket packet;
            if (pointcloud_queue.try_dequeue(packet)) {
                // 处理点云数据（使用shared_ptr，零拷贝）
                // std::cout << "收到点云数据包，大小: " << packet.data->size() << " 字节" << std::endl;
                // 处理完毕后，packet.data析构，引用计数-1
                // 如果引用计数归1，内存池可以复用该缓冲区
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    // 消费IMU线程
    std::thread imu_consumer_thread([&running]() {
        auto& imu_queue = free_queue::SharedQueues::instance().imuQueue();
        while (running.load()) {
            free_queue::IMUPacket packet;
            if (imu_queue.try_dequeue(packet)) {
                // 处理IMU数据（使用shared_ptr，零拷贝）
                // std::cout << "收到IMU数据包，大小: " << packet.data->size() << " 字节" << std::endl;
                // 处理完毕后，packet.data析构，引用计数-1
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    });

    // 等待用户中断
    std::cout << "按Enter键退出..." << std::endl;
    std::cin.get();

    running.store(false);

    // 断开所有雷达连接
    for (auto& lidar : lidar_devices) {
        lidar->disconnect();  // 使用 -> 因为是指针
    }

    consumer_thread.join();
    pointcloud_consumer_thread.join();
    imu_consumer_thread.join();  // 释放work_guard，允许io_context退出
    work_guard.reset();
    // 等待所有线程
    for (auto& t : io_threads) { t.join(); }
    return 0;
}