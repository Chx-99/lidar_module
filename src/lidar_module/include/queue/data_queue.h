#pragma once

#include "concurrentqueue.h"

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace free_queue {

// ACK数据包（小包，保持直接拷贝）
struct AckPacket {
    std::string sn;             // 雷达序列号
    std::vector<uint8_t> data;  // ACK数据
};

// 点云数据包（大包，使用shared_ptr零拷贝传递）
struct PointCloudPacket {
    std::string sn;                              // 雷达序列号
    std::shared_ptr<std::vector<uint8_t>> data;  // 点云数据（共享指针）
};

// IMU数据包（中等大小，使用shared_ptr提高效率）
struct IMUPacket {
    std::string sn;                              // 雷达序列号
    std::shared_ptr<std::vector<uint8_t>> data;  // IMU数据（共享指针）
};

// 独立雷达的队列集合
struct LidarQueues {
    // 队列配置
    static constexpr size_t ACK_QUEUE_INITIAL_SIZE = 16;            // 每个雷达的ACK队列容量
    static constexpr size_t POINTCLOUD_QUEUE_INITIAL_SIZE = 4096;   // 每个雷达的点云队列容量
    static constexpr size_t IMU_QUEUE_INITIAL_SIZE = 512;           // 每个雷达的IMU队列容量

    moodycamel::ConcurrentQueue<AckPacket> ack_queue;
    moodycamel::ConcurrentQueue<PointCloudPacket> pointcloud_queue;
    moodycamel::ConcurrentQueue<IMUPacket> imu_queue;

    LidarQueues()
        : ack_queue(ACK_QUEUE_INITIAL_SIZE),
          pointcloud_queue(POINTCLOUD_QUEUE_INITIAL_SIZE),
          imu_queue(IMU_QUEUE_INITIAL_SIZE) {}

    // 获取队列大小
    size_t ackQueueSize() const { return ack_queue.size_approx(); }
    size_t pointCloudQueueSize() const { return pointcloud_queue.size_approx(); }
    size_t imuQueueSize() const { return imu_queue.size_approx(); }
};

// 队列管理器：管理多个独立雷达的队列
// 使用读写锁（shared_mutex）实现高性能并发访问
// 读操作：多个线程可并发读取（shared_lock）
// 写操作：独占访问（unique_lock）
class QueueManager {
public:
    using QueuesMap = std::unordered_map<std::string, std::shared_ptr<LidarQueues>>;

    static QueueManager& instance() {
        static QueueManager instance;
        return instance;
    }

    // 雷达申请队列（如果不存在则创建）返回队列的共享指针
    std::shared_ptr<LidarQueues> registerLidar(const std::string& sn) {
        // 先尝试读取（快速路径：大多数情况下雷达已注册）
        {
            std::shared_lock<std::shared_mutex> read_lock(mutex_);
            auto it = queues_.find(sn);
            if (it != queues_.end()) {
                return it->second;  // 已存在，直接返回
            }
        }
        
        // 不存在，需要创建（写操作）
        std::unique_lock<std::shared_mutex> write_lock(mutex_);
        
        // 双重检查：可能在等待锁期间已被其他线程创建
        auto it = queues_.find(sn);
        if (it != queues_.end()) {
            return it->second;
        }
        
        // 创建新队列并插入（try_emplace 避免不必要的构造）
        auto [inserted_it, success] = queues_.try_emplace(
            sn, 
            std::make_shared<LidarQueues>()
        );
        
        return inserted_it->second;
    }

    // 注销雷达（移除队列）
    void unregisterLidar(const std::string& sn) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        queues_.erase(sn);
    }

    // 获取雷达队列（多线程并发读取）
    std::shared_ptr<LidarQueues> getLidarQueues(const std::string& sn) const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        
        auto it = queues_.find(sn);
        if (it != queues_.end()) {
            return it->second;  // shared_ptr 复制，引用计数安全
        }
        return nullptr;
    }

    // 获取所有雷达队列的快照（复制 shared_ptr，不复制队列本身）
    QueuesMap getQueuesSnapshot() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return queues_;  // 复制 map，但内部的 shared_ptr 只增加引用计数
    }

    // 获取所有雷达SN列表
    std::vector<std::string> getAllLidarSNs() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        
        std::vector<std::string> sns;
        sns.reserve(queues_.size());
        for (const auto& [sn, _] : queues_) {
            sns.push_back(sn);
        }
        return sns;
    }

    // 获取注册的雷达数量
    size_t getLidarCount() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return queues_.size();
    }

private:
    QueueManager() = default;
    ~QueueManager() = default;
    QueueManager(const QueueManager&) = delete;
    QueueManager& operator=(const QueueManager&) = delete;

    // 雷达队列映射表
    QueuesMap queues_;
    
    // 读写锁：允许多个读者并发，写者独占
    mutable std::shared_mutex mutex_;
};

}  // namespace free_queue
