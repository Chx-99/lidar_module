#pragma once

#include "base/data_struct.h"
#include "memory/buffer_pool.h"
#include "network/port_scan.h"
#include "queue/data_queue.h"

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <tuple>

namespace lidar_base
{
    using namespace base_frame;
    using namespace free_queue;

    class Lidar
    {
    public:
        Lidar(boost::asio::io_context &io_context, std::string lidar_ip, std::string local_ip, std::string sn)
            : lidar_ip_(std::move(lidar_ip)),
              local_ip_(std::move(local_ip)),
              sn_(std::move(sn)),
              io_context_(io_context),
              cmd_io_context_(),                                              // 命令独立的 io_context
              cmd_work_guard_(boost::asio::make_work_guard(cmd_io_context_)), // 保持io_context运行
              cmd_socket_(cmd_io_context_),                                   // 命令 socket 使用独立的 io_context
              imu_socket_(io_context),
              pointcloud_socket_(io_context),
              heartbeat_timer_(cmd_io_context_),
              reconnect_timer_(cmd_io_context_) // 重连定时器
        {
            // 注册雷达，获取独立队列
            queues_ = QueueManager::instance().registerLidar(sn_);

            // 初始化ProducerToken（提升入队性能）
            pointcloud_producer_token_ = std::make_unique<moodycamel::ProducerToken>(queues_->pointcloud_queue);
            imu_producer_token_ = std::make_unique<moodycamel::ProducerToken>(queues_->imu_queue);

            std::cout << "雷达 " << sn_ << " 已注册队列" << std::endl;

            // 分配端口
            try
            {
                auto ports = network_tools::getAvailablePorts(3);
                std::tie(cmd_port_, imu_port_, pointcloud_port_) = std::make_tuple(ports[0], ports[1], ports[2]);
                std::cout << "为雷达 " << sn_ << " 分配端口: cmd=" << static_cast<int>(cmd_port_)
                          << ", imu=" << static_cast<int>(imu_port_)
                          << ", pointcloud=" << static_cast<int>(pointcloud_port_) << std::endl;

                remote_endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(lidar_ip_), 65000);

                // 初始化所有socket
                initSockets();

                // ACK包较小，保留临时缓冲区用于同步等待机制
                ack_recv_buffer_.reserve(1024);

                // 启动命令线程（专门处理同步命令收发）
                cmd_thread_ = std::thread([this]()
                                          { cmd_io_context_.run(); });

                // 注意：不在构造函数中调用connect()
                // 需要在io_context.run()启动后再调用connect()
            }
            catch (const boost::system::system_error &e)
            {
                std::cerr << "雷达 " << sn_ << " 初始化失败: " << e.what() << std::endl;
                std::cerr << "详细信息: 本地IP=" << local_ip_ << ", 雷达IP=" << lidar_ip_
                          << ", cmd端口=" << static_cast<int>(cmd_port_) << ", imu端口=" << static_cast<int>(imu_port_)
                          << ", 点云端口=" << static_cast<int>(pointcloud_port_) << std::endl;
                throw;
            }
            catch (const std::exception &e)
            {
                std::cerr << "雷达 " << sn_ << " 初始化失败: " << e.what() << std::endl;
                throw;
            }
        }
        ~Lidar()
        {
            disconnect();

            // 注销雷达队列
            QueueManager::instance().unregisterLidar(sn_);

            // 停止命令 io_context 并等待线程结束
            cmd_work_guard_.reset(); // 释放work_guard，允许io_context退出
            cmd_io_context_.stop();  // 停止io_context
            if (cmd_thread_.joinable())
            {
                cmd_thread_.join();
            }
        }

        // 连接雷达（发送初始化命令序列）
        // 返回: true=成功, false=失败
        bool connect()
        {
            std::cout << "雷达 " << sn_ << " 开始连接..." << std::endl;

            // 1. 发送握手指令并等待ACK
            base_frame::Frame<base_frame::HandShake> handshake_frame(local_ip_, pointcloud_port_, cmd_port_, imu_port_);
            if (!sendCommandAndWaitAck(FRAME_TO_SPAN(handshake_frame)))
            {
                std::cerr << "雷达 " << sn_ << " 握手失败" << std::endl;
                return false;
            }

            // 2. 发送开启激光指令并等待ACK
            base_frame::Frame<base_frame::SetLaserStatus> laser_frame(0x01);
            if (!sendCommandAndWaitAck(FRAME_TO_SPAN(laser_frame)))
            {
                std::cerr << "雷达 " << sn_ << " 开启激光失败" << std::endl;
                return false;
            }

            // 3. 发送设置IMU频率指令并等待ACK
            base_frame::Frame<base_frame::SetIMUFrequency> imu_frame(0x01);
            if (!sendCommandAndWaitAck(FRAME_TO_SPAN(imu_frame)))
            {
                std::cerr << "雷达 " << sn_ << " 设置IMU频率失败" << std::endl;
                return false;
            }

            // 初始化成功，设置为运行状态
            is_running_.store(true, std::memory_order_release);

            // 启动数据接收和心跳监控
            startReceivePointCloud();
            startReceiveIMU();
            startHeartbeat();

            std::cout << "雷达 " << sn_ << " 连接成功" << std::endl;
            return true;
        }

        // 断开连接（可重复调用，线程安全）
        void disconnect()
        {
            // 防止重复日志输出和状态变更
            bool expected = true;
            if (!is_running_.compare_exchange_strong(expected, false))
            {
                return; // 已经是断开状态
            }

            std::cout << "雷达 " << sn_ << " 主动断开连接" << std::endl;

            // 取消所有定时器（会中断pending的异步等待）
            heartbeat_timer_.cancel();
            reconnect_timer_.cancel();

            // 关闭所有socket（会自动取消pending的异步操作）
            closeAllSockets();

            // 清理缓冲区并释放内存
            ack_recv_buffer_.clear();
            ack_recv_buffer_.shrink_to_fit();
        }

        // 重置socket（仅在断开状态下调用）
        bool reset()
        {
            // 安全检查：只允许在断开状态下重置
            if (is_running_.load(std::memory_order_acquire))
            {
                std::cerr << "雷达 " << sn_ << " 重置失败: 必须先调用disconnect()" << std::endl;
                return false;
            }

            try
            {
                // 确保所有socket已关闭（幂等操作）
                closeAllSockets();

                // 重新打开并配置所有socket
                initSockets();

                // 重置ACK缓冲区
                ack_recv_buffer_.clear();
                ack_recv_buffer_.reserve(1024);

                return true;
            }
            catch (const std::exception &e)
            {
                std::cerr << "雷达 " << sn_ << " 重置socket失败: " << e.what() << std::endl;
                return false;
            }
        }

        // 发送命令并同步等待ACK（已在构造函数中设置3秒超时）
        bool sendCommandAndWaitAck(std::span<const uint8_t> frame)
        {
            std::lock_guard<std::mutex> lock(send_mutex_); // 发送命令加锁，防止并发发送导致ACK错乱
            try
            {
                cmd_socket_.send_to(boost::asio::buffer(frame.data(), frame.size()), remote_endpoint_);
                std::cout << "发送命令:" << GET_ACK_NAME_COMPILE_TIME(frame.data()) << std::endl;
            }
            catch (const boost::system::system_error &e)
            {
                throw std::runtime_error("发送命令失败: " + std::string(e.what()));
            }

            try
            {
                size_t bytes_received = cmd_socket_.receive(boost::asio::buffer(ack_recv_buffer_.data(), ack_recv_buffer_.capacity()));
                std::cout << "收到ACK:" << GET_ACK_NAME_COMPILE_TIME(ack_recv_buffer_.data()) << std::endl;
                return GET_ACK_RET_CODE(ack_recv_buffer_) == 0x00;
            }
            catch (const boost::system::system_error &e)
            {
                // 检查是否是超时错误
                if (e.code() == boost::asio::error::would_block || e.code() == boost::asio::error::timed_out)
                {
                    throw std::runtime_error("接收ACK超时（3秒内未收到响应）");
                }
                throw std::runtime_error("接收ACK失败: " + std::string(e.what()));
            }
        }

    private:
        // 初始化所有socket（构造和reset共用）
        void initSockets()
        {
            auto interface_name = network_tools::getInterfaceNameFromIp(local_ip_);

            // 初始化 cmd_socket
            cmd_socket_.open(boost::asio::ip::udp::v4());
            cmd_socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(local_ip_), cmd_port_));
            cmd_socket_.set_option(boost::asio::socket_base::broadcast(true));
            setsockopt(cmd_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(), interface_name.size());
            struct timeval tv{cmd_timeout_.count(), 0};
            setsockopt(cmd_socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            // 初始化 imu_socket
            imu_socket_.open(boost::asio::ip::udp::v4());
            imu_socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), imu_port_));
            imu_socket_.set_option(boost::asio::socket_base::reuse_address(true));
            setsockopt(imu_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(), interface_name.size());

            // 初始化 pointcloud_socket
            pointcloud_socket_.open(boost::asio::ip::udp::v4());
            pointcloud_socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), pointcloud_port_));
            pointcloud_socket_.set_option(boost::asio::socket_base::reuse_address(true));
            setsockopt(pointcloud_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(), interface_name.size());
        }

        // 关闭所有socket（幂等操作，可安全重复调用）
        void closeAllSockets()
        {
            boost::system::error_code ec;
            cmd_socket_.close(ec);
            imu_socket_.close(ec);
            pointcloud_socket_.close(ec);
        }

        // 心跳定时器回调
        void heartbeatTimerCallback()
        {
            if (!is_running_.load(std::memory_order_acquire))
            {
                return; // 已断开，停止心跳
            }

            try
            {
                // 发送心跳并等待ACK
                base_frame::Frame<base_frame::HeartBeat> heartbeat_frame;
                bool success = sendCommandAndWaitAck(FRAME_TO_SPAN(heartbeat_frame));

                if (!success)
                {
                    std::cerr << "雷达 " << sn_ << " 心跳应答失败，切换到重连模式" << std::endl;

                    // 设置断开状态（停止所有异步接收）
                    is_running_.store(false, std::memory_order_release);

                    // 关闭所有socket（触发异步操作结束）
                    closeAllSockets();

                    // 切换到重连定时器
                    startReconnectTimer();
                    return;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "雷达 " << sn_ << " 心跳异常: " << e.what() << "，切换到重连模式" << std::endl;

                // 设置断开状态
                is_running_.store(false, std::memory_order_release);

                // 关闭所有socket
                closeAllSockets();

                // 切换到重连定时器
                startReconnectTimer();
                return;
            }

            // 心跳成功，继续下一次
            heartbeat_timer_.expires_after(std::chrono::seconds(1));
            heartbeat_timer_.async_wait([this](const boost::system::error_code &ec)
                                        {
            if (!ec) { heartbeatTimerCallback(); } });
        }

        void startHeartbeat()
        {
            heartbeat_timer_.expires_after(std::chrono::seconds(1));
            heartbeat_timer_.async_wait([this](const boost::system::error_code &ec)
                                        {
            if (!ec) { heartbeatTimerCallback(); } });
        }

        // 启动重连定时器
        void startReconnectTimer()
        {
            reconnect_timer_.expires_after(reconnect_interval_);
            reconnect_timer_.async_wait([this](const boost::system::error_code &ec)
                                        {
                if (ec) return;  // 定时器被取消
                
                std::cout << "雷达 " << sn_ << " 尝试重连..." << std::endl;
                
                // 重置 socket
                if (!reset()) {
                    std::cerr << "雷达 " << sn_ << " 重置失败，" << reconnect_interval_.count() << "秒后重试" << std::endl;
                    startReconnectTimer();  // 继续重连定时器
                    return;
                }
                
                // 尝试重新连接
                if (connect()) {
                    std::cout << "雷达 " << sn_ << " 重连成功，切换到心跳模式" << std::endl;
                    // 重连成功，切换到心跳定时器（connect() 内部会调用 startHeartbeat()）
                } else {
                    std::cerr << "雷达 " << sn_ << " 重连失败，" << reconnect_interval_.count() << "秒后重试" << std::endl;
                    startReconnectTimer();  // 继续重连定时器
                } });
        }

        // 点云接收（直接用内存池buffer接收，零拷贝）
        void startReceivePointCloud()
        {
            // 从内存池获取buffer，直接用于接收网络数据
            auto buffer = pointcloud_buffer_pool_.acquire();

            // 确保有足够容量（首次或降级分配时）
            if (buffer->capacity() < 2048)
            {
                buffer->reserve(2048);
            }
            buffer->resize(buffer->capacity()); // 使用全部可用空间接收

            pointcloud_socket_.async_receive_from(
                boost::asio::buffer(buffer->data(), buffer->size()), sender_endpoint_,
                [this, buffer](boost::system::error_code ec, std::size_t bytes_received)
                {
                    if (!ec && bytes_received > 0 && is_running_.load(std::memory_order_acquire))
                    {
                        // 调整buffer到实际接收的大小
                        buffer->resize(bytes_received);

                        // 直接入队，零拷贝传递（shared_ptr），enqueue会自动唤醒等待线程
                        queues_->pointcloud_queue.enqueue(*pointcloud_producer_token_, buffer);
                    }

                    if (pointcloud_socket_.is_open() && is_running_.load(std::memory_order_acquire))
                    {
                        startReceivePointCloud();
                    }
                });
        }

        // IMU接收（直接用内存池buffer接收，零拷贝）
        void startReceiveIMU()
        {
            // 从内存池获取buffer，直接用于接收网络数据
            auto buffer = imu_buffer_pool_.acquire();

            // 确保有足够容量（首次或降级分配时）
            if (buffer->capacity() < 1024)
            {
                buffer->reserve(1024);
            }
            buffer->resize(buffer->capacity()); // 使用全部可用空间接收

            imu_socket_.async_receive_from(boost::asio::buffer(buffer->data(), buffer->size()), sender_endpoint_,
                                           [this, buffer](boost::system::error_code ec, std::size_t bytes_received)
                                           {
                                               if (!ec && bytes_received > 0 &&
                                                   is_running_.load(std::memory_order_acquire))
                                               {
                                                   // 调整buffer到实际接收的大小
                                                   buffer->resize(bytes_received);

                                                   // 直接入队，零拷贝传递（shared_ptr），enqueue会自动唤醒等待线程
                                                   queues_->imu_queue.enqueue(*imu_producer_token_, buffer);
                                               }

                                               if (imu_socket_.is_open() && is_running_.load(std::memory_order_acquire))
                                               {
                                                   startReceiveIMU();
                                               }
                                           });
        }

    private:
        // 基础数据成员（无依赖）- 必须先声明，因为在构造函数中先被move
        std::string lidar_ip_; // 雷达ip
        std::string local_ip_; // 本地ip
        std::string sn_;       // 雷达序列号

        std::mutex send_mutex_; // 发送命令互斥锁

        // io_context 引用（被其他成员依赖，需要先声明）
        boost::asio::io_context &io_context_;                                                     // 主 io_context（点云/IMU/心跳）
        boost::asio::io_context cmd_io_context_;                                                  // 命令独立 io_context
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> cmd_work_guard_; // 保持cmd_io_context运行
        std::thread cmd_thread_;                                                                  // 命令线程

        // 端口句柄（RAII 管理，自动释放）
        uint16_t cmd_port_;        // 命令端口
        uint16_t pointcloud_port_; // 数据端口
        uint16_t imu_port_;        // IMU端口

        // 网络组件（依赖 io_context 和端口）
        boost::asio::ip::udp::socket cmd_socket_;        // 用于接收与发送命令
        boost::asio::ip::udp::socket imu_socket_;        // 用于接收IMU数据
        boost::asio::ip::udp::socket pointcloud_socket_; // 用于接收点云数据
        boost::asio::ip::udp::endpoint remote_endpoint_; // 远程端点

        // 定时器（依赖 io_context）
        boost::asio::steady_timer heartbeat_timer_; // 心跳定时器
        boost::asio::steady_timer reconnect_timer_; // 重连定时器

        // 接收缓冲区
        std::vector<uint8_t> ack_recv_buffer_; // 用于存储接收到的ACK数据

        // 发送方端点（用于接收数据时记录来源）
        boost::asio::ip::udp::endpoint sender_endpoint_;

        // 独立队列引用（由QueueManager管理）
        std::shared_ptr<LidarQueues> queues_;

        // ProducerToken（提升入队性能，需要在获取队列后初始化）
        std::unique_ptr<moodycamel::ProducerToken> pointcloud_producer_token_;
        std::unique_ptr<moodycamel::ProducerToken> imu_producer_token_;

        // 内存池（零拷贝传递，避免频繁堆分配）
        memory_pool::BufferPool<uint8_t, 32> pointcloud_buffer_pool_; // 点云数据内存池（大包，高频）
        memory_pool::BufferPool<uint8_t, 16> imu_buffer_pool_;        // IMU数据内存池（中等频率）

        // 运行状态
        std::atomic<bool> is_running_{false};

        // 重连时间间隔
        static constexpr std::chrono::seconds reconnect_interval_{20};
        // cmd超时时间
        static constexpr std::chrono::seconds cmd_timeout_{2};
    };
}; // namespace lidar_base