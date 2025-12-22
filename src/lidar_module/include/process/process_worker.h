#pragma once

#include "queue/data_queue.h"
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>
#include <vector>

namespace process_worker {
    using namespace free_queue;

// 线程池工作模式：轮询所有雷达队列
class ProcessorThreadPool {
public:
    using AckCallback = std::function<void(const std::string& sn, const AckPacket&)>;
    using PointCloudCallback = std::function<void(const std::string& sn, const PointCloudPacket&)>;
    using IMUCallback = std::function<void(const std::string& sn, const IMUPacket&)>;

    explicit ProcessorThreadPool(size_t thread_count = 4)
        : thread_count_(thread_count), running_(false) {}

    ~ProcessorThreadPool() {
        stop();
    }

    // 设置处理回调
    void setAckCallback(AckCallback cb) {
        ack_callback_ = std::move(cb);
    }

    void setPointCloudCallback(PointCloudCallback cb) {
        pointcloud_callback_ = std::move(cb);
    }

    void setIMUCallback(IMUCallback cb) {
        imu_callback_ = std::move(cb);
    }

    // 启动线程池
    void start() {
        if (running_) return;
        
        running_ = true;
        for (size_t i = 0; i < thread_count_; ++i) {
            threads_.emplace_back(&ProcessorThreadPool::workerLoop, this, i);
        }
        std::cout << "Thread pool started with " << thread_count_ << " threads\n";
    }

    // 停止线程池
    void stop() {
        if (!running_) return;
        
        running_ = false;
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        threads_.clear();
        std::cout << "Thread pool stopped\n";
    }

private:
    // 工作线程循环：轮询所有雷达队列
    void workerLoop(size_t thread_id) {
        std::cout << "Worker thread " << thread_id << " started\n";

        // 每个线程的批量出队缓冲区（减少系统调用）
        constexpr size_t BATCH_SIZE = 16;
        std::vector<AckPacket> ack_batch(BATCH_SIZE);
        std::vector<PointCloudPacket> pc_batch(BATCH_SIZE);
        std::vector<IMUPacket> imu_batch(BATCH_SIZE);

        // 缓存雷达SN列表（雷达注册/注销是低频操作，减少刷新频率）
        std::vector<std::string> cached_sns;
        size_t refresh_counter = 0;
        constexpr size_t REFRESH_INTERVAL = 1000;  // 每1000次循环刷新一次（降低10倍）

        // 连续空闲计数器，用于自适应休眠
        size_t idle_count = 0;
        constexpr size_t IDLE_THRESHOLD = 5;  // 连续5次空闲后增加休眠时间

        while (running_) {
            // 定期刷新雷达列表（避免复制整个map，只复制SN列表）
            if (refresh_counter++ % REFRESH_INTERVAL == 0) {
                cached_sns = QueueManager::instance().getAllLidarSNs();
                if (cached_sns.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
            }

            bool has_work = false;

            // 轮询每个雷达的队列（按需获取，避免复制整个map）
            for (const auto& sn : cached_sns) {
                // 按需获取雷达队列（shared_lock，开销小）
                auto queues = QueueManager::instance().getLidarQueues(sn);
                if (!queues) continue;  // 雷达可能已注销
                
                // 批量处理ACK数据
                size_t ack_count = queues->ack_queue.try_dequeue_bulk(ack_batch.begin(), BATCH_SIZE);
                if (ack_count > 0 && ack_callback_) {
                    has_work = true;
                    for (size_t i = 0; i < ack_count; ++i) {
                        ack_callback_(sn, ack_batch[i]);
                    }
                }

                // 批量处理点云数据
                size_t pc_count = queues->pointcloud_queue.try_dequeue_bulk(pc_batch.begin(), BATCH_SIZE);
                if (pc_count > 0 && pointcloud_callback_) {
                    has_work = true;
                    for (size_t i = 0; i < pc_count; ++i) {
                        pointcloud_callback_(sn, pc_batch[i]);
                    }
                }

                // 批量处理IMU数据
                size_t imu_count = queues->imu_queue.try_dequeue_bulk(imu_batch.begin(), BATCH_SIZE);
                if (imu_count > 0 && imu_callback_) {
                    has_work = true;
                    for (size_t i = 0; i < imu_count; ++i) {
                        imu_callback_(sn, imu_batch[i]);
                    }
                }
            }

            // 自适应休眠：增加休眠时间，降低CPU占用
            if (!has_work) {
                idle_count++;
                if (idle_count < IDLE_THRESHOLD) {
                    // 短期空闲：1毫秒休眠（从500微秒增加）
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                } else {
                    // 长期空闲：5毫秒休眠（从1毫秒增加）
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            } else {
                // 有工作时重置空闲计数器，但仍短暂yield避免霸占CPU
                idle_count = 0;
                std::this_thread::yield();  // 让出时间片，降低CPU占用
            }
        }

        std::cout << "Worker thread " << thread_id << " stopped\n";
    }

    size_t thread_count_;
    std::atomic<bool> running_;
    std::vector<std::thread> threads_;
    AckCallback ack_callback_;
    PointCloudCallback pointcloud_callback_;
    IMUCallback imu_callback_;
};

}  // namespace free_queue
