#pragma once

#include "concurrentqueue.h"
#include <string>
#include <vector>
#include <cstdint>

namespace free_queue
{
    // ACK数据包
    struct AckPacket
    {
        std::string sn;                // 雷达序列号
        std::vector<uint8_t> data;     // ACK数据
    };

    // 点云数据包
    struct PointCloudPacket
    {
        std::string sn;                // 雷达序列号
        std::vector<uint8_t> data;     // 点云数据
    };

    // IMU数据包
    struct IMUPacket
    {
        std::string sn;                // 雷达序列号
        std::vector<uint8_t> data;     // IMU数据
    };

    // 全局共享队列（多台雷达公用）
    class SharedQueues
    {
    public:
        // 队列配置
        static constexpr size_t ACK_QUEUE_INITIAL_SIZE = 256;           // ACK队列初始容量
        static constexpr size_t POINTCLOUD_QUEUE_INITIAL_SIZE = 512;    // 点云队列初始容量
        static constexpr size_t IMU_QUEUE_INITIAL_SIZE = 1024;           // IMU队列初始容量（频率较高）

        static SharedQueues& instance()
        {
            static SharedQueues instance;
            return instance;
        }

        // 获取队列引用
        moodycamel::ConcurrentQueue<AckPacket>& ackQueue() { return ack_queue_; }
        moodycamel::ConcurrentQueue<PointCloudPacket>& pointCloudQueue() { return pointcloud_queue_; }
        moodycamel::ConcurrentQueue<IMUPacket>& imuQueue() { return imu_queue_; }

        // 获取队列当前大小（近似值，仅用于监控）
        size_t ackQueueSize() const { return ack_queue_.size_approx(); }
        size_t pointCloudQueueSize() const { return pointcloud_queue_.size_approx(); }
        size_t imuQueueSize() const { return imu_queue_.size_approx(); }

    private:
        SharedQueues() 
            : ack_queue_(ACK_QUEUE_INITIAL_SIZE),
              pointcloud_queue_(POINTCLOUD_QUEUE_INITIAL_SIZE),
              imu_queue_(IMU_QUEUE_INITIAL_SIZE)
        {
        }
        
        ~SharedQueues() = default;
        SharedQueues(const SharedQueues&) = delete;
        SharedQueues& operator=(const SharedQueues&) = delete;

        moodycamel::ConcurrentQueue<AckPacket> ack_queue_;
        moodycamel::ConcurrentQueue<PointCloudPacket> pointcloud_queue_;
        moodycamel::ConcurrentQueue<IMUPacket> imu_queue_;
    };

} // namespace free_queue
