#pragma once

#include "queue/data_queue.h"
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace process_worker
{
    using namespace free_queue;

    // 点云批量收集配置
    struct BatchConfig
    {
        size_t pointcloud_batch_size = 2500;          // 批量大小：收集多少个包后处理
        int64_t pointcloud_timeout_ms = 1000;         // 超时：超过此时间未满也处理
        static constexpr size_t imu_batch_size = 200; // IMU批量大小
        static constexpr int64_t imu_timeout_ms = 5;  // IMU超时

        // 根据采集频率计算配置
        static BatchConfig fromFrequency(double frequency_hz)
        {
            BatchConfig config;
            config.pointcloud_batch_size = static_cast<size_t>(2500.0 / frequency_hz);
            config.pointcloud_timeout_ms = static_cast<int64_t>(1000.0 / frequency_hz);
            return config;
        }
    };

    // 雷达线程信息
    struct LidarThreadInfo
    {
        std::thread thread;
        std::atomic<bool> should_stop{false};
        BatchConfig config; // 每个雷达独立的点云批量配置
    };

    // 线程池工作模式：每个雷达独立线程 + 阻塞队列
    class ProcessorThreadPool
    {
    public:
        explicit ProcessorThreadPool()
            : running_(false) {}

        ~ProcessorThreadPool()
        {
            stop();
        }

        // 设置处理回调
        void setPointCloudBatchCallback(PointCloudBatchCallback cb) { pointcloud_batch_callback_ = std::move(cb); }
        void setIMUCallback(IMUCallback cb) { imu_callback_ = std::move(cb); }

        // 根据采集频率设置全局默认点云批量配置（推荐使用此方法）
        void setDefaultPointCloudFrequency(double frequency_hz)
        {
            default_pc_batch_config_ = BatchConfig::fromFrequency(frequency_hz);
            std::cout << "默认点云批量配置: batch_size=" << default_pc_batch_config_.pointcloud_batch_size
                      << ", timeout_ms=" << default_pc_batch_config_.pointcloud_timeout_ms << "\n";
        }

        // 为指定雷达设置点云批量配置
        void setLidarPointCloudBatchConfig(const std::string &sn, const BatchConfig &config)
        {
            auto it = lidar_threads_.find(sn);
            if (it != lidar_threads_.end())
            {
                it->second.config = config;
                std::cout << "雷达 " << sn << " 点云批量配置: batch_size=" << config.pointcloud_batch_size
                          << ", timeout_ms=" << config.pointcloud_timeout_ms << "\n";
            }
        }

        // 根据采集频率为指定雷达设置点云批量配置
        void setLidarPointCloudFrequency(const std::string &sn, double frequency_hz)
        {
            setLidarPointCloudBatchConfig(sn, BatchConfig::fromFrequency(frequency_hz));
        }

        // 启动线程池（为每个已注册的雷达创建处理线程）
        void start()
        {
            if (running_)
                return;
            running_ = true;

            // 获取所有雷达SN
            auto sns = QueueManager::instance().getAllLidarSNs();
            for (const auto &sn : sns)
            {
                startLidarProcessor(sn);
            }

            std::cout << "线程池已启动，包含 " << lidar_threads_.size() << " 个雷达处理器\n";
        }

        // 动态添加雷达处理线程（雷达注册后调用）
        void addLidar(const std::string &sn)
        {
            if (!running_)
                return;
            startLidarProcessor(sn);
        }

        // 移除雷达处理线程（雷达注销前调用）
        void removeLidar(const std::string &sn)
        {
            auto it = lidar_threads_.find(sn);
            if (it != lidar_threads_.end())
            {
                it->second.should_stop = true;
                if (it->second.thread.joinable())
                {
                    it->second.thread.join();
                }
                lidar_threads_.erase(it);
            }
        }

        // 停止线程池
        void stop()
        {
            if (!running_)
                return;
            running_ = false;

            // 停止所有雷达处理线程
            for (auto &[sn, thread_info] : lidar_threads_)
            {
                thread_info.should_stop = true;
            }

            for (auto &[sn, thread_info] : lidar_threads_)
            {
                if (thread_info.thread.joinable())
                {
                    thread_info.thread.join();
                }
            }

            lidar_threads_.clear();
            std::cout << "线程池已停止\n";
        }

    private:
        // 启动单个雷达的处理线程
        void startLidarProcessor(const std::string &sn)
        {
            auto queues = QueueManager::instance().getLidarQueues(sn);
            if (!queues)
                return;

            // 使用 emplace 直接构造，避免 atomic 的移动赋值问题
            auto [it, inserted] = lidar_threads_.try_emplace(sn);
            if (inserted)
            {
                it->second.config = default_pc_batch_config_; // 使用默认配置
                it->second.thread = std::thread(&ProcessorThreadPool::lidarWorkerLoop, this, sn, queues);
            }

            std::cout << "已启动雷达处理器: " << sn << "\n";
        }

        // 单个雷达的工作线程（3个子线程分别处理3个队列）
        void lidarWorkerLoop(const std::string &sn, std::shared_ptr<LidarQueues> queues)
        {
            auto &thread_info = lidar_threads_[sn];

            std::thread pointcloud_thread([this, sn, queues, &thread_info]()
                                          { processPointCloudQueue(sn, queues, thread_info.should_stop); });

            std::thread imu_thread([this, sn, queues, &thread_info]()
                                   { processIMUQueue(sn, queues, thread_info.should_stop); });

            // 等待所有子线程完成
            pointcloud_thread.join();
            imu_thread.join();

            std::cout << "雷达 " << sn << " 处理器已停止\n";
        }

        // 点云队列处理：批量收集后处理（使用wait_dequeue_bulk_timed高效批量获取）
        void processPointCloudQueue(const std::string &sn, std::shared_ptr<LidarQueues> queues, std::atomic<bool> &should_stop)
        {
            // 获取该雷达的配置
            auto & config = lidar_threads_[sn].config;
            std::vector<std::shared_ptr<std::vector<uint8_t>>> batch;
            batch.resize(config.pointcloud_batch_size); // 使用雷达独立的配置

            while (running_ && !should_stop)
            {
                size_t count = queues->pointcloud_queue.try_dequeue_bulk(batch.begin(), config.pointcloud_batch_size);
                // 批量回调处理
                if (count > 0 && pointcloud_batch_callback_)
                {
                    // 调整batch大小为实际获取的数量，避免多余的复制
                    batch.resize(count);
                    pointcloud_batch_callback_(sn, batch);
                    // 恢复batch大小以供下次使用（shared_ptr自动管理引用计数）
                    batch.resize(config.pointcloud_batch_size);
                }
                else
                {
                    // 队列为空时才休眠，有数据时立即处理，避免积压
                    std::this_thread::sleep_for(std::chrono::milliseconds(config.pointcloud_timeout_ms));
                }
            }
        }

        // IMU队列处理：单个实时处理
        void processIMUQueue(const std::string &sn, std::shared_ptr<LidarQueues> queues, std::atomic<bool> &should_stop)
        {
            auto & config = lidar_threads_[sn].config;
            while (running_ && !should_stop)
            {
                // std::cout << queues->imuQueueSize() << std::endl;

                std::shared_ptr<std::vector<uint8_t>> packet;
                // 阻塞等待，带超时（用于检查退出标志）
                if (queues->imu_queue.try_dequeue(packet))
                {
                    if (imu_callback_ && packet)
                    {
                        imu_callback_(sn, *packet);
                    }
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(config.imu_timeout_ms)); // 防止忙等待
                }
            }
        }

        std::atomic<bool> running_;
        std::unordered_map<std::string, LidarThreadInfo> lidar_threads_; // 每个雷达的线程

        // 回调函数
        PointCloudBatchCallback pointcloud_batch_callback_;
        IMUCallback imu_callback_;

        // 默认点云批量配置（新注册雷达使用）
        BatchConfig default_pc_batch_config_;
    };

} // namespace process_worker
