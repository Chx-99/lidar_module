#include "base/data_struct.h"
#include "network/port_scan.h"
#include "concurrentqueue.h"
#include "lidar/lidar_base.h"
#include "queue/data_queue.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>
#include <thread>

using namespace std::chrono;

// 定义时间测量宏
#define MEASURE_TIME(operation, iterations, name)                                                               \
    auto start_##name = high_resolution_clock::now();                                                           \
    for (int i = 0; i < iterations; ++i)                                                                        \
    {                                                                                                           \
        operation;                                                                                              \
    }                                                                                                           \
    auto end_##name = high_resolution_clock::now();                                                             \
    auto duration_##name = duration_cast<nanoseconds>(end_##name - start_##name);                               \
    std::cout << #name << " (" << iterations << " iterations) 执行时间: " << duration_##name.count() << " 纳秒" \
              << std::endl;                                                                                     \
    std::cout << "平均每次执行时间: " << duration_##name.count() / iterations << " 纳秒" << std::endl;

using namespace lidar_module;
using namespace base_frame;
using namespace frame_tools;

int main(int argc, char **argv)
{
    boost::asio::io_context io_context;

    // 创建work_guard保持io_context运行（全局管理）
    auto work_guard = boost::asio::make_work_guard(io_context);

    Lidar lidar(io_context, "192.168.1.139", "192.168.1.100", "SN123456", 10);

    // 在后台线程运行io_context
    std::thread io_thread([&io_context]()
                          { io_context.run(); });

    // 等待io_context启动
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 现在调用connect
    try
    {
        lidar.connect();
        std::cout << "雷达连接成功！" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "雷达连接失败: " << e.what() << std::endl;
        io_context.stop();
        io_thread.join();
        return 1;
    }

    // 启动消费者线程处理ACK队列
    std::atomic<bool> running{true};
    std::thread consumer_thread([&running]()
                                {
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
                    HeartBeatACK heartbeat = fromVector<HeartBeatACK>(packet.data);
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
        } });

    // 消费点云线程
    std::thread pointcloud_consumer_thread([&running]()
                                           {
        auto& pointcloud_queue = free_queue::SharedQueues::instance().pointCloudQueue();
        while (running.load()) {
            free_queue::PointCloudPacket packet;
            if (pointcloud_queue.try_dequeue(packet)) {
                // 处理点云数据
                // std::cout << "收到点云数据包，大小: " << packet.data.size() << " 字节" << std::endl;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } });

    // 消费IMU线程
    std::thread imu_consumer_thread([&running]()
                                    {
        auto& imu_queue = free_queue::SharedQueues::instance().imuQueue();
        while (running.load()) {
            free_queue::IMUPacket packet;
            if (imu_queue.try_dequeue(packet)) {
                // 处理IMU数据
                // std::cout << "收到IMU数据包，大小: " << packet.data.size() << " 字节" << std::endl;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        } });

    // 等待用户中断
    std::cout << "按Enter键退出..." << std::endl;
    std::cin.get();

    running.store(false);
    lidar.disconnect();
    consumer_thread.join();

    // 释放work_guard，允许io_context退出
    work_guard.reset();
    io_thread.join();

    return 0;
}