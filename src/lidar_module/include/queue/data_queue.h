#pragma once

#include "concurrentqueue.h"

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace free_queue
{

    struct LidarQueues
    {
        // 队列配置（降低容量，避免ConcurrentQueue内部分配过多内存块）
        static constexpr size_t ack_queue_max_size_ = 16;          // 每个雷达的ACK队列容量
        static constexpr size_t pointcloud_queue_max_size_ = 512;  // 降低到512，快速消费避免积压
        static constexpr size_t imu_queue_max_size_ = 128;         // 降低到128

        // 阻塞队列：wait_dequeue 会自动阻塞等待数据，无需轮询
        moodycamel::ConcurrentQueue<std::shared_ptr<std::vector<uint8_t>>> ack_queue;
        moodycamel::ConcurrentQueue<std::shared_ptr<std::vector<uint8_t>>> pointcloud_queue;
        moodycamel::ConcurrentQueue<std::shared_ptr<std::vector<uint8_t>>> imu_queue;

        LidarQueues()
            : ack_queue(ack_queue_max_size_), 
            pointcloud_queue(pointcloud_queue_max_size_), imu_queue(imu_queue_max_size_) {}

        // 获取队列大小
        size_t ackQueueSize() const { return ack_queue.size_approx(); }
        size_t pointCloudQueueSize() const { return pointcloud_queue.size_approx(); }
        size_t imuQueueSize() const { return imu_queue.size_approx(); }
    };

    // 队列管理器：管理多个独立雷达的队列
    class QueueManager
    {
    public:

        static QueueManager &instance()
        {
            static QueueManager instance;
            return instance;
        }

        // 雷达申请队列（如果不存在则创建）返回队列的共享指针
        std::shared_ptr<LidarQueues> registerLidar(const std::string &sn)
        {
            {
                std::shared_lock<std::shared_mutex> read_lock(mutex_);
                auto it = queues_.find(sn);
                if (it != queues_.end())
                {
                    return it->second; // 已存在，直接返回
                }
            }

            // 不存在，需要创建（写操作）
            std::unique_lock<std::shared_mutex> write_lock(mutex_);

            // 双重检查：可能在等待锁期间已被其他线程创建
            auto it = queues_.find(sn);
            if (it != queues_.end())
            {
                return it->second;
            }

            // 创建新队列并插入（try_emplace 避免不必要的构造）
            auto [inserted_it, success] = queues_.try_emplace(
                sn,
                std::make_shared<LidarQueues>());

            return inserted_it->second;
        }

        // 注销雷达（移除队列）
        void unregisterLidar(const std::string &sn)
        {
            std::unique_lock<std::shared_mutex> lock(mutex_);
            queues_.erase(sn);
        }

        // 获取雷达队列
        std::shared_ptr<LidarQueues> getLidarQueues(const std::string &sn) const
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);

            auto it = queues_.find(sn);
            if (it != queues_.end())
            {
                return it->second; // shared_ptr 复制，引用计数安全
            }
            return nullptr;
        }

        // 获取所有雷达队列的快照
        std::unordered_map<std::string, std::shared_ptr<LidarQueues>> getQueuesSnapshot() const
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return queues_; // 复制 map，但内部的 shared_ptr 只增加引用计数
        }

        // 获取所有雷达SN列表
        std::vector<std::string> getAllLidarSNs() const
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);

            std::vector<std::string> sns;
            sns.reserve(queues_.size());
            for (const auto &[sn, _] : queues_)
            {
                sns.push_back(sn);
            }
            return sns;
        }

        // 获取注册的雷达数量
        size_t getLidarCount() const
        {
            std::shared_lock<std::shared_mutex> lock(mutex_);
            return queues_.size();
        }

    private:
        QueueManager() = default;
        ~QueueManager() = default;
        QueueManager(const QueueManager &) = delete;
        QueueManager &operator=(const QueueManager &) = delete;

        // 雷达队列映射表
        std::unordered_map<std::string, std::shared_ptr<LidarQueues>> queues_;

        // 读写锁：允许多个读者并发，写者独占
        mutable std::shared_mutex mutex_;
    };

} // namespace free_queue
