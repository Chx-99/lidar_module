#pragma once

#include <boost/asio.hpp>

#include "sensor/lidar/base/data_struct.hpp"
#include "sensor/lidar/base/port_scan.hpp"
#include "blockingconcurrentqueue.h"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace lidar_base
{
    using namespace lidar_frame_tools;
    using namespace network_tools;
    using namespace lidar_base_frame;

    enum class LidarState
    {
        CLOSED,       // 关闭
        DISCONNECTED, // 未连接
        IDLE,         // 空闲（心跳）
        COLLECTING,   // 采集中
    };

    /**
     * @brief 雷达基类
     *
     * 职责：
     * 1. 管理网络连接（自动连接、心跳、重连）
     * 2. 提供基础的命令收发接口
     * 3. 定义采集接口（由子类实现具体逻辑）
     */
    class LidarBase
    {

    public:
        /**
         * @brief 构造函数
         */
        LidarBase(const std::string lidar_ip, const std::string local_ip, const std::string sn, size_t package_num = 2500, bool repetitive_scan = false)
            : sn_(std::move(sn)), lidar_ip_(std::move(lidar_ip)), local_ip_(std::move(local_ip)),
              package_num_(package_num),
              repetitive_scan_(repetitive_scan),
              base_strand_(base_io_context_.get_executor()),
              ack_strand_(ack_io_context_.get_executor()),
              base_work_guard_(boost::asio::make_work_guard(base_io_context_)),
              ack_work_guard_(boost::asio::make_work_guard(ack_io_context_)),
              receive_work_guard_(boost::asio::make_work_guard(receive_io_context_)),
              pointcloud_recv_buffer_(2048), imu_recv_buffer_(1024), ack_recv_buffer_(512),
              batch_buffer_(2500), pointcloud_queue_(4096), imu_queue_(1024), state_(LidarState::DISCONNECTED)
        {
            init();
            open_sockets();
            connect();
        }

        ~LidarBase()
        {

            state_.store(LidarState::CLOSED, std::memory_order_release);

            // 2. 取消所有定时器，防止心跳继续发送命令
            if (heartbeat_timer_)
            {
                heartbeat_timer_->cancel();
            }
            if (reconnect_timer_)
            {
                reconnect_timer_->cancel();
            }

            // 3. 显式取消所有 socket 上的异步操作
            boost::system::error_code ec;
            if (cmd_socket_ && cmd_socket_->is_open())
            {
                cmd_socket_->cancel(ec);
            }
            if (pointcloud_socket_ && pointcloud_socket_->is_open())
            {
                pointcloud_socket_->cancel(ec);
            }
            if (imu_socket_ && imu_socket_->is_open())
            {
                imu_socket_->cancel(ec);
            }

            // 4. 关闭所有socket，确保资源释放
            close_sockets();

            // 5. 释放work_guard，让io_context可以退出
            base_work_guard_.reset();
            ack_work_guard_.reset();
            receive_work_guard_.reset();

            // 6. 停止io_context（不会中断正在执行的handler）
            base_io_context_.stop();
            ack_io_context_.stop();
            receive_io_context_.stop();

            // 7. 等待所有线程结束（确保所有回调都执行完毕）
            for (auto &thread : io_threads_)
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }
        }

        // 禁止拷贝和移动
        LidarBase(const LidarBase &) = delete;
        LidarBase &operator=(const LidarBase &) = delete;

        /**
         * @brief 获取当前状态
         */
        LidarState getState() const { return state_.load(std::memory_order_acquire); }

        /**
         * @brief 获取雷达SN
         */
        const std::string &getSN() const { return sn_; }
        /**
         * @brief 获取一帧点云数据
         */
        sensor_msgs::msg::PointCloud2::SharedPtr getPointcloudData(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
        {
            if (batch_buffer_.capacity() < package_num_)
            {
                batch_buffer_.reserve(package_num_);
            }
            batch_buffer_.resize(package_num_);

            size_t total_received = 0;
            while (total_received < package_num_)
            {
                auto length = pointcloud_queue_.wait_dequeue_bulk_timed(batch_buffer_.begin() + total_received, package_num_ - total_received, timeout);
                if (length == 0)
                {
                    RCLCPP_WARN(rclcpp::get_logger("lidar_base"), "[%s] getPointcloudData 超时：期望 %zu 帧, 已接收 %zu 帧, 超时 %ld ms",
                                sn_.c_str(), package_num_, total_received, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
                    return nullptr;
                }
                total_received += length;
            }
            return lidar_frame_tools::convertToPointCloud2(batch_buffer_, sn_);
        }

        /**
         * @brief 获得一帧imu数据
         */
        sensor_msgs::msg::Imu::SharedPtr getImuData(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
        {
            DataFrame<ImuData, 1> frame;
            if (!imu_queue_.wait_dequeue_timed(frame, timeout))
            {
                RCLCPP_WARN(rclcpp::get_logger("lidar_base"), "[%s] getImuData 超时：超时 %ld ms",
                            sn_.c_str(), std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
                return nullptr;
            }
            // 使用DataFrame内的timestamp
            return lidar_frame_tools::convertToImu(frame, sn_);
        }

        /**
         * @brief 开启激光
         */
        bool enableLaser(int retry_count = 5)
        {
            if (state_.load(std::memory_order_acquire) < LidarState::IDLE)
            {
                return false;
            }
            
            Frame<SetLaserStatus> frame(0x01);
            for (int retry = 0; retry < retry_count; retry++)
            {
                
                sendCommand(frameToSpan(frame));
                if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(frame)), std::chrono::seconds(5)))
                {
                    startReceive();
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief 关闭激光
         */
        bool disableLaser(int retry_count = 5)
        {
            if (state_.load(std::memory_order_acquire) < LidarState::IDLE)
            {
                return false;
            }
            Frame<SetLaserStatus> frame(0x00);
            for (int retry = 0; retry < retry_count; retry++)
            {
                sendCommand(frameToSpan(frame));
                if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(frame)), std::chrono::seconds(5)))
                {
                    stopReceive();
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief 开启IMU
         */
        bool enableIMU(int retry_count = 5)
        {
            if (state_.load(std::memory_order_acquire) < LidarState::IDLE)
            {
                return false;
            }
            Frame<SetIMUFrequency> frame(0x01);
            for (int retry = 0; retry < retry_count; retry++)
            {
                sendCommand(frameToSpan(frame));
                if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(frame)), std::chrono::seconds(5)))
                {
                    return true;
                }
            }
            return false;
        }

        /**
         * @brief 关闭IMU
         */
        bool disableIMU(int retry_count = 5)
        {
            if (state_.load(std::memory_order_acquire) < LidarState::IDLE)
            {
                return false;
            }
            Frame<SetIMUFrequency> frame(0x00);
            for (int retry = 0; retry < retry_count; retry++)
            {
                sendCommand(frameToSpan(frame));
                if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(frame)), std::chrono::seconds(5)))
                {
                    return true;
                }
            }
            return false;
        }

    private:
        /**
         * @brief 发送命令
         */
        void sendCommand(std::span<const uint8_t> data)
        {
            std::lock_guard<std::mutex> lock(send_mutex_);
            try
            {
                cmd_socket_->async_send_to(boost::asio::buffer(data.data(), data.size()), sender_endpoint_, [](const boost::system::error_code &ec, size_t bytes_transferred)
                {
                    if (ec)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "发送超时/失败: %s", ec.message().c_str());
                    }
                    else{
                        RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "发送命令成功, 字节数: %zu", bytes_transferred);
                    }
                });
            }
            catch (const boost::system::system_error &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "[%s] 发送超时/失败: %s", sn_.c_str(), e.what());
            }
        }

        void startReceive()
        {
            // 只有在IDLE状态才能开始接收
            LidarState expected = LidarState::IDLE;
            if (!state_.compare_exchange_strong(expected, LidarState::COLLECTING, std::memory_order_acq_rel))
            {
                return; // 已经在接收中或状态不正确
            }
            receive_pointcloud();
            receive_imu();
        }
        /**
         * @brief 停止数据接收
         */
        void stopReceive()
        {
            // 只有在COLLECTING状态才需要停止
            LidarState expected = LidarState::COLLECTING;
            if (!state_.compare_exchange_strong(expected, LidarState::IDLE, std::memory_order_acq_rel))
            {
                return; // 已经停止了或状态不正确
            }

            lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96> pc_item;
            while (pointcloud_queue_.try_dequeue(pc_item))
                ;
            lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1> imu_item;
            while (imu_queue_.try_dequeue(imu_item))
                ;

            // 取消异步接收操作
            boost::system::error_code ec;
            if (pointcloud_socket_ && pointcloud_socket_->is_open())
            {
                pointcloud_socket_->cancel(ec);
            }
            if (imu_socket_ && imu_socket_->is_open())
            {
                imu_socket_->cancel(ec);
            }
        }

        /**
         * @brief 连接雷达
         */
        bool connect()
        {
            if (state_.load(std::memory_order_acquire) >= LidarState::IDLE)
            {
                return true; // 已经连接了
            }

            receive_ack();

            Frame<HandShake> handshake_frame(local_ip_, pointcloud_port_, cmd_port_, imu_port_);
            sendCommand(frameToSpan(handshake_frame));
            if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(handshake_frame)), std::chrono::seconds(5)))
            {
                // Frame<WriteDeviceParam> write_id_frame(0x02, 0x01, 0x00);
                // if (repetitive_scan_){
                //     write_id_frame = Frame<WriteDeviceParam>(0x02, 0x01, 0x01);
                //     RCLCPP_INFO(rclcpp::get_logger("lidar_base"), "[%s] 设置扫描参数: 重复扫描", sn_.c_str());
                // } else {
                //     write_id_frame = Frame<WriteDeviceParam>(0x02, 0x01, 0x00);
                //     RCLCPP_INFO(rclcpp::get_logger("lidar_base"), "[%s] 设置扫描参数: 非重复扫描", sn_.c_str());
                // }

                // sendCommand(frameToSpan(write_id_frame));
                // if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(write_id_frame)), std::chrono::seconds(5)))
                // {
                //     RCLCPP_INFO(rclcpp::get_logger("lidar_base"), "[%s] 设置扫描参数失败", sn_.c_str());
                // } else {
                //     RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "[%s] 设置扫描参数失败", sn_.c_str());
                // }
                state_.store(LidarState::IDLE, std::memory_order_release);
                heartbeat();
                return true;
            }
            return false;
        }

        /**
         * @brief 断开连接
         */
        bool disconnect()
        {
            if (state_.load(std::memory_order_acquire) <= LidarState::DISCONNECTED)
            {
                return true; // 已经断开了
            }

            // 先停止接收
            stopReceive();

            Frame<Disconnect> disconnect_frame;
            sendCommand(frameToSpan(disconnect_frame));
            if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(disconnect_frame)), std::chrono::seconds(5)))
            {
                state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                return true;
            }
            return false;
        }
        /**
         * @brief 启动数据接收
         */

        void receive_ack()
        {
            cmd_socket_->async_receive_from(boost::asio::buffer(ack_recv_buffer_.data(), ack_recv_buffer_.capacity()), remote_endpoint_,
                                            [this](const boost::system::error_code &ec, std::size_t)
                                            {
                                                // 检查对象状态，防止访问已销毁的对象
                                                if (ec || state_.load(std::memory_order_acquire) == LidarState::CLOSED)
                                                {
                                                    return;
                                                }

                                                uint16_t ack_set_id = GET_ACK_SETID(ack_recv_buffer_);
                                                bool ack_ret = (GET_ACK_RET_CODE(ack_recv_buffer_) == 0x00);
                                                {
                                                    std::lock_guard<std::mutex> lock(ack_mutex_);
                                                    ack_map_[ack_set_id] = ack_ret;
                                                }
                                                ack_cv_.notify_all();
                                                receive_ack();
                                            });
        }

        void receive_pointcloud()
        {
            pointcloud_socket_->async_receive_from(boost::asio::buffer(pointcloud_recv_buffer_.data(), pointcloud_recv_buffer_.capacity()), remote_endpoint_,
                                                   [this](const boost::system::error_code &ec, std::size_t bytes_transferred)
                                                   {
                                                       // 先检查对象状态，防止访问已销毁的对象
                                                       if (ec || state_.load(std::memory_order_acquire) < LidarState::COLLECTING)
                                                       {
                                                           return;
                                                       }

                                                       constexpr size_t expected_size = sizeof(DataFrame<SingleEchoRectangularData, 96>);
                                                       if (bytes_transferred == expected_size && pointcloud_queue_.size_approx() < package_num_)
                                                       {
                                                           DataFrame<SingleEchoRectangularData, 96> frame;
                                                           std::memcpy(&frame, pointcloud_recv_buffer_.data(), expected_size);
                                                           if (!pointcloud_queue_.enqueue(std::move(frame)))
                                                           {
                                                               RCLCPP_WARN(rclcpp::get_logger("lidar_base"), "[%s] pointcloud_queue丢弃一帧点云", sn_.c_str());
                                                           }
                                                           else{
                                                                RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "[%s] 接收点云数据包: %zu bytes", sn_.c_str(), bytes_transferred);
                                                           }
                                                           
                                                       }
                                                       receive_pointcloud();
                                                   });
        }

        void receive_imu()
        {

            imu_socket_->async_receive_from(boost::asio::buffer(imu_recv_buffer_.data(), imu_recv_buffer_.capacity()), remote_endpoint_,
                                            [this](const boost::system::error_code &ec, std::size_t bytes_transferred)
                                            {
                                                // 先检查对象状态，防止访问已销毁的对象
                                                if (ec || state_.load(std::memory_order_acquire) < LidarState::COLLECTING)
                                                {
                                                    return;
                                                }
                                                constexpr size_t expected_size = sizeof(DataFrame<ImuData, 1>);
                                                if (bytes_transferred == expected_size)
                                                {
                                                    DataFrame<ImuData, 1> imu_frame;
                                                    std::memcpy(&imu_frame, imu_recv_buffer_.data(), expected_size);
                                                    if (!imu_queue_.enqueue(std::move(imu_frame)))
                                                    {
                                                        RCLCPP_WARN(rclcpp::get_logger("lidar_base"), "[%s] imu_queue丢弃一帧IMU数据", sn_.c_str());
                                                    }
                                                }
                                                // 无论数据是否正确，都继续接收
                                                receive_imu();
                                            });
        }
        void heartbeat()
        {
            heartbeat_timer_->expires_after(std::chrono::seconds(1));
            heartbeat_timer_->async_wait([this](const boost::system::error_code &ec)
                                         {
                                            if (ec || state_.load(std::memory_order_acquire) < LidarState::IDLE)
                                            {
                                                return;
                                            }
                                            try
                                            {
                                                Frame<HeartBeat> heartbeat_frame;
                                                sendCommand(frameToSpan(heartbeat_frame));
                                                if (wait_and_get_ack(GET_ACK_SETID(frameToSpan(heartbeat_frame)), std::chrono::seconds(10)))
                                                {
                                                    heartbeat();
                                                }
                                                else
                                                {

                                                    reconnect();
                                                    return;
                                                }
                                            }
                                            catch(...) {
                                                reconnect();
                                            } });
        }

        /**
         * @brief 启动重连定时器
         */
        void reconnect()
        {
            RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ]心跳ACK失败，准备重连", sn_.c_str());
            state_.store(LidarState::DISCONNECTED, std::memory_order_release);
            reconnect_timer_->expires_after(reconnect_interval_);
            reconnect_timer_->async_wait([this](const boost::system::error_code &ec)
                                         {
                                             // 如果出现错误或已经关闭，则不执行重连
                                             if (ec || state_.load(std::memory_order_acquire) == LidarState::CLOSED)
                                             {
                                                 return;
                                             }
                                             try
                                             {
                                                 close_sockets();
                                                 open_sockets();
                                                 // 重新启动ACK接收（重连时必须重新启动）
                                                 receive_ack();
                                                 if (connect())
                                                 {
                                                     // 重连成功，停止定时器
                                                     return;
                                                 }
                                                 else{
                                                    // 重连失败，继续尝试
                                                    reconnect();
                                                 }
                                             }
                                             catch (...)
                                             {
                                                 // 重连失败，继续尝试
                                                 reconnect();
                                             } });
        }

        /**
         * @brief 获取ack回应
         */
        bool wait_and_get_ack(uint16_t set_id, std::chrono::seconds timeout)
        {
            std::unique_lock<std::mutex> lock(ack_mutex_);
            if (!ack_cv_.wait_for(lock, timeout, [this, &set_id]
                                  { return ack_map_.count(set_id) > 0; }))
            {
                return false; // 超时
            }
            // 获取ack-bool，并从map中移除
            bool ack_success = ack_map_[set_id];
            ack_map_.erase(set_id);
            return ack_success;
        }

        /**
         * @brief 打开所有socket
         */
        void open_sockets()
        {
            auto interface_name = network_tools::getInterfaceNameFromIp(local_ip_);
            // 初始化 cmd_socket
            cmd_socket_->open(boost::asio::ip::udp::v4());
            cmd_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(local_ip_), cmd_port_));
            cmd_socket_->set_option(boost::asio::socket_base::broadcast(true));
            setsockopt(cmd_socket_->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
                       interface_name.size());
            struct timeval tv{cmd_timeout_.count(), 0};
            setsockopt(cmd_socket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            // 初始化 imu_socket
            imu_socket_->open(boost::asio::ip::udp::v4());
            imu_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), imu_port_));
            imu_socket_->set_option(boost::asio::socket_base::reuse_address(true));
            setsockopt(imu_socket_->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
                       interface_name.size());
            // 初始化 pointcloud_socket
            pointcloud_socket_->open(boost::asio::ip::udp::v4());
            pointcloud_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), pointcloud_port_));
            pointcloud_socket_->set_option(boost::asio::socket_base::reuse_address(true));
            setsockopt(pointcloud_socket_->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
                       interface_name.size());
        }

        /**
         * @brief 关闭所有socket
         */
        void close_sockets()
        {
            boost::system::error_code ec;
            if (cmd_socket_ && cmd_socket_->is_open())
            {
                cmd_socket_->close(ec);
            }
            if (imu_socket_ && imu_socket_->is_open())
            {
                imu_socket_->close(ec);
            }
            if (pointcloud_socket_ && pointcloud_socket_->is_open())
            {
                pointcloud_socket_->close(ec);
            }
        }

        void init()
        {
            // 初始化socket
            cmd_socket_ = std::make_unique<boost::asio::ip::udp::socket>(ack_io_context_);
            imu_socket_ = std::make_unique<boost::asio::ip::udp::socket>(receive_io_context_);
            pointcloud_socket_ = std::make_unique<boost::asio::ip::udp::socket>(receive_io_context_);
            heartbeat_timer_ = std::make_unique<boost::asio::steady_timer>(base_io_context_);
            reconnect_timer_ = std::make_unique<boost::asio::steady_timer>(base_io_context_);
            sender_endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(lidar_ip_), 65000);
            auto ports = network_tools::getAvailablePorts(3);
            std::tie(cmd_port_, imu_port_, pointcloud_port_) = std::make_tuple(ports[0], ports[1], ports[2]);
            // std::cout << "为雷达 " << sn_ << " 分配端口: cmd=" << static_cast<int>(cmd_port_)
            //           << ", imu=" << static_cast<int>(imu_port_) << ", pointcloud=" << static_cast<int>(pointcloud_port_)
            //           << std::endl;

            io_threads_.emplace_back([this]()
                                     { base_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { ack_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { receive_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { receive_io_context_.run(); });
            RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ] 分配端口: cmd=%u, imu=%u, pointcloud=%u",
                         sn_.c_str(), cmd_port_, imu_port_, pointcloud_port_);
        }

    protected:
        std::string sn_;

    private:
        std::string lidar_ip_;     // 雷达IP地址
        std::string local_ip_;     // 本地IP地址
        uint16_t cmd_port_;        // 命令端口
        uint16_t pointcloud_port_; // 数据端口
        uint16_t imu_port_;        // IMU端口

        size_t package_num_;       // 每帧点云包含的数据包数量
        bool repetitive_scan_; // 是否重复扫描

        std::mutex ack_mutex_, send_mutex_; // 互斥锁
        std::condition_variable ack_cv_;    // 条件变量

        boost::asio::io_context base_io_context_, ack_io_context_, receive_io_context_;                                                  // 心跳和重连使用base_io_context_，数据接收使用receive_io_context_
        boost::asio::strand<boost::asio::io_context::executor_type> base_strand_, ack_strand_;                                       // 用于保护心跳和重连相关的状态
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> base_work_guard_, ack_work_guard_, receive_work_guard_; // 用于保持io_context运行

        boost::asio::ip::udp::endpoint remote_endpoint_;                  // 远程端点
        boost::asio::ip::udp::endpoint sender_endpoint_;                  // 发送端点
        std::unique_ptr<boost::asio::ip::udp::socket> cmd_socket_;        // 命令socket
        std::unique_ptr<boost::asio::ip::udp::socket> pointcloud_socket_; // 点云socket
        std::unique_ptr<boost::asio::ip::udp::socket> imu_socket_;        // IMU socket
        std::unique_ptr<boost::asio::steady_timer> heartbeat_timer_;      // 心跳定时器
        std::unique_ptr<boost::asio::steady_timer> reconnect_timer_;      // 重连定时器

        std::vector<std::thread> io_threads_; // asio工作线程

        std::vector<uint8_t> pointcloud_recv_buffer_; // 用于存储接收到的点云数据
        std::vector<uint8_t> imu_recv_buffer_;        // 用于存储接收到的IMU数据
        std::vector<uint8_t> ack_recv_buffer_;        // 用于存储接收到的ACK数据
        std::unordered_map<uint16_t, bool> ack_map_;  // 存储ACK结果的映射表

        std::vector<DataFrame<SingleEchoRectangularData, 96>> batch_buffer_;                             // 批量点云数据缓存
        moodycamel::BlockingConcurrentQueue<DataFrame<SingleEchoRectangularData, 96>> pointcloud_queue_; // 点云数据队列
        moodycamel::BlockingConcurrentQueue<DataFrame<ImuData, 1>> imu_queue_;                           // IMU数据队列

        rclcpp::Clock clock_{RCL_ROS_TIME}; // ROS时钟

        // 状态
        std::atomic<LidarState> state_{LidarState::CLOSED};
        // 重连时间间隔
        static constexpr std::chrono::seconds reconnect_interval_{20};
        // cmd超时时间
        static constexpr std::chrono::seconds cmd_timeout_{2};
    };

} // namespace lidar_base
