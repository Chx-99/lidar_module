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
    /**
     * @brief 雷达状态
     */
    enum class LidarState
    {
        DISCONNECTED, // 未连接
        IDLE,         // 空闲（已连接）
        COLLECTING,   // 采集中
        ERROR         // 错误
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
        LidarBase(const std::string lidar_ip, const std::string local_ip, const std::string sn)
            : sn_(std::move(sn)), lidar_ip_(std::move(lidar_ip)), local_ip_(std::move(local_ip)),
              cmd_strand_(hr_io_context_.get_executor()),
              hr_work_guard_(boost::asio::make_work_guard(hr_io_context_)),
              cpi_work_guard_(boost::asio::make_work_guard(cpi_io_context_)),
              ack_recv_buffer_(512), pointcloud_recv_buffer_(2048), imu_recv_buffer_(1024),
              state_(LidarState::DISCONNECTED), is_running_(false), is_receiving_(false)
        {
            batch_buffer_.reserve(2500);
            init();
            openSockets();
            startReceiveACK();
            connect();
        }

        ~LidarBase()
        {
            // 1. 先设置标志位，停止所有新操作
            is_receiving_.store(false, std::memory_order_release);
            is_running_.store(false, std::memory_order_release);
            state_.store(LidarState::DISCONNECTED, std::memory_order_release);

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
            closeSockets();

            // 5. 释放work_guard，让io_context可以退出
            hr_work_guard_.reset();
            cpi_work_guard_.reset();

            // 6. 停止io_context（不会中断正在执行的handler）
            hr_io_context_.stop();
            cpi_io_context_.stop();

            // 7. 等待所有线程结束（确保所有回调都执行完毕）
            for (auto &thread : io_threads_)
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }

            // 8. 线程安全：此时所有异步操作已完成，可以安全清空队列
            lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96> pc_item;
            while (pointcloud_queue_.try_dequeue(pc_item))
                ;
            lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1> imu_item;
            while (imu_queue_.try_dequeue(imu_item))
                ;
            std::pair<uint16_t, bool> ack_item;
            while (ack_queue_.try_dequeue(ack_item))
                ;
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
        sensor_msgs::msg::PointCloud2::SharedPtr getPointcloudData(size_t package_num, std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
        {
            if (batch_buffer_.capacity() < package_num) {
                batch_buffer_.reserve(package_num);
            }
            batch_buffer_.resize(package_num);
            size_t total_received = 0;
            while (total_received < package_num)
            {
                auto length = pointcloud_queue_.wait_dequeue_bulk_timed(batch_buffer_.begin() + total_received, package_num - total_received, timeout);
                if (length == 0)
                {
                    RCLCPP_WARN(rclcpp::get_logger("lidar_base"), "[%s] getPointcloudData 超时：期望 %zu 帧, 已接收 %zu 帧, 超时 %ld ms",
                                sn_.c_str(), package_num, total_received, std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
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
            lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1> frame;
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
            if (!is_running_.load(std::memory_order_acquire))
            {
                return false;
            }
            bool flag = false;
            lidar_base_frame::Frame<lidar_base_frame::SetLaserStatus> frame(0x01);
            for (int retry = 0; retry < retry_count; retry++)
            {
                flag = sendCommand(frameToSpan(frame));
                if (flag)
                {
                    startReceive();
                    break;
                }
            }
            return flag;
        }

        /**
         * @brief 关闭激光
         */
        bool disableLaser(int retry_count = 5)
        {
            if (!is_running_.load(std::memory_order_acquire))
            {
                return false;
            }
            bool flag = false;
            lidar_base_frame::Frame<lidar_base_frame::SetLaserStatus> frame(0x00);
            for (int retry = 0; retry < retry_count; retry++)
            {
                flag = sendCommand(frameToSpan(frame));
                if (flag)
                {
                    stopReceive();
                    break;
                }
            }
            return flag;
        }

        /**
         * @brief 开启IMU
         */
        bool enableIMU(int retry_count = 5)
        {
            if (!is_running_.load(std::memory_order_acquire))
            {
                return false;
            }
            bool flag = false;
            lidar_base_frame::Frame<lidar_base_frame::SetIMUFrequency> frame(0x01);
            for (int retry = 0; retry < retry_count; retry++)
            {
                flag = sendCommand(frameToSpan(frame));
                if (flag)
                {
                    break;
                }
            }
            return flag;
        }

        /**
         * @brief 关闭IMU
         */
        bool disableIMU(int retry_count = 5)
        {
            if (!is_running_.load(std::memory_order_acquire))
            {
                return false;
            }
            bool flag = false;
            lidar_base_frame::Frame<lidar_base_frame::SetIMUFrequency> frame(0x00);
            for (int retry = 0; retry < retry_count; retry++)
            {
                flag = sendCommand(frameToSpan(frame));
                if (flag)
                {
                    break;
                }
            }
            return flag;
        }

    private:
        /**
         * @brief 发送命令并等待ACK
         */
        bool sendCommand(std::span<const uint8_t> data)
        {
            std::lock_guard<std::mutex> lock(send_mutex_);
            try
            {
                cmd_socket_->send_to(boost::asio::buffer(data.data(), data.size()), sender_endpoint_);
                RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ] 发送命令: %s", sn_.c_str(), GET_ACK_NAME(data).data());
            }
            catch (const boost::system::system_error &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "[%s] 发送超时/失败: %s", sn_.c_str(), e.what());
                return false;
            }

            // 等待ACK
            std::pair<uint16_t, bool> ack_data;
            if (ack_queue_.wait_dequeue_timed(ack_data, std::chrono::milliseconds(1000)))
            {
                if (ack_data.first != GET_ACK_SETID(data))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "[%s] 接收ACK错误: 期望 %u, 实际 %u", sn_.c_str(), GET_ACK_SETID(data), ack_data.first);
                    return false;
                }
                RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ] 接收ACK: %s", sn_.c_str(), GET_ACK_NAME(data).data());
                return ack_data.second;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "[%s] 接收ACK超时", sn_.c_str());
                return false;
            }
        }

        void startReceive()
        {
            bool expected = false;
            if (!is_receiving_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
            {
                return; // 已经在接收中
            }
            state_.store(LidarState::COLLECTING, std::memory_order_release);
            startReceivePointcloud();
            startReceiveIMU();
        }
        /**
         * @brief 停止数据接收
         */
        void stopReceive()
        {
            bool expected = true;
            if (!is_receiving_.compare_exchange_strong(expected, false, std::memory_order_acq_rel))
            {
                return; // 已经停止了
            }

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

            // 清空队列
            lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96> pc_item;
            while (pointcloud_queue_.try_dequeue(pc_item))
                ;
            lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1> imu_item;
            while (imu_queue_.try_dequeue(imu_item))
                ;

            // 只有在连接状态下才设置为IDLE，否则保持DISCONNECTED
            if (is_running_.load(std::memory_order_acquire))
            {
                state_.store(LidarState::IDLE, std::memory_order_release);
            }
        }

        /**
         * @brief 连接雷达
         */
        bool connect()
        {
            bool expected = false;
            if (!is_running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
            {
                return true; // 已经连接了
            }
            
            auto guard = [this](int*) { 
                is_running_.store(false, std::memory_order_release); 
            };
            std::unique_ptr<int, decltype(guard)> flag_guard(nullptr, guard);
            
            lidar_base_frame::Frame<lidar_base_frame::HandShake> handshake_frame(local_ip_, pointcloud_port_, cmd_port_, imu_port_);
            if (sendCommand(frameToSpan(handshake_frame)))
            {
                flag_guard.release(); // 连接成功，不重置标志位
                state_.store(LidarState::IDLE, std::memory_order_release);
                startHeartbeat();
                return true;
            }
            return false;
        }

        /**
         * @brief 断开连接
         */
        bool disconnect()
        {
            bool expected = true;
            if (!is_running_.compare_exchange_strong(expected, false, std::memory_order_acq_rel))
            {
                return true; // 已经断开了
            }

            // 保存接收状态，以便失败时恢复
            bool was_receiving = is_receiving_.load(std::memory_order_acquire);
            
            // 先停止接收
            stopReceive();

            lidar_base_frame::Frame<lidar_base_frame::Disconnect> disconnect_frame;
            if (sendCommand(frameToSpan(disconnect_frame)))
            {
                is_receiving_.store(false, std::memory_order_release);
                state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                return true;
            }
            
            // 断开失败，恢复所有标志位
            is_running_.store(true, std::memory_order_release);
            if (was_receiving)
            {
                // 尝试恢复接收状态（不保证成功）
                startReceive();
            }
            return false;
        }
        /**
         * @brief 启动数据接收
         */
        void startReceivePointcloud()
        {
            if (!is_receiving_.load(std::memory_order_acquire) || !is_running_.load(std::memory_order_acquire))
            {
                return;
            }
            pointcloud_socket_->async_receive_from(
                boost::asio::buffer(pointcloud_recv_buffer_.data(), pointcloud_recv_buffer_.capacity()),
                remote_endpoint_,
                [this](const boost::system::error_code &ec, std::size_t bytes_transferred)
                {
                    // 先检查对象状态，防止访问已销毁的对象
                    if (!is_receiving_.load(std::memory_order_acquire) || !is_running_.load(std::memory_order_acquire))
                    {
                        return;
                    }
                    
                    // 检查错误码
                    if (ec)
                    {
                        // 网络错误或socket已关闭，停止接收
                        return;
                    }
                    
                    constexpr size_t expected_size = sizeof(lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96>);
                    
                    if (bytes_transferred == expected_size)
                    {
                        // 数据正确，解析并入队
                        lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96> frame;
                        std::memcpy(&frame, pointcloud_recv_buffer_.data(), expected_size);
                        frame.timestamp = clock_.now().nanoseconds();
                        pointcloud_queue_.try_enqueue(std::move(frame));
                    }
                    // 无论数据是否正确，都继续接收
                    // 使用post异步调度下一次接收，避免在回调中直接调用导致栈溢出和CPU过载
                    boost::asio::post(pointcloud_socket_->get_executor(), 
                        [this]() { startReceivePointcloud(); });
                });
        }

        void startReceiveACK()
        {
            cmd_socket_->async_receive_from(
                boost::asio::buffer(ack_recv_buffer_.data(), ack_recv_buffer_.capacity()),
                remote_endpoint_,
                [this](const boost::system::error_code &ec, std::size_t)
                {
                    // 检查对象状态，防止访问已销毁的对象
                    if (!is_running_.load(std::memory_order_acquire))
                    {
                        return;
                    }
                    
                    // 如果有错误，停止接收
                    if (ec)
                    {
                        return;
                    }

                    std::pair<uint16_t, bool> ack_data;
                    ack_data.first = GET_ACK_SETID(ack_recv_buffer_);
                    ack_data.second = (GET_ACK_RET_CODE(ack_recv_buffer_) == 0x00);
                    ack_queue_.try_enqueue(std::move(ack_data));

                    // 使用post异步调度下一次接收
                    boost::asio::post(cmd_socket_->get_executor(), 
                        [this]() { startReceiveACK(); });
                });
        }

        void startReceiveIMU()
        {
            if (!is_receiving_.load(std::memory_order_acquire) || !is_running_.load(std::memory_order_acquire))
            {
                return;
            }
            imu_socket_->async_receive_from(
                boost::asio::buffer(imu_recv_buffer_.data(), imu_recv_buffer_.capacity()),
                remote_endpoint_,
                [this](const boost::system::error_code &ec, std::size_t bytes_transferred)
                {
                    // 先检查对象状态，防止访问已销毁的对象
                    if (!is_receiving_.load(std::memory_order_acquire) || !is_running_.load(std::memory_order_acquire))
                    {
                        return;
                    }
                    
                    // 检查错误码
                    if (ec)
                    {
                        // 网络错误或socket已关闭，停止接收
                        return;
                    }
                    
                    constexpr size_t expected_size = sizeof(lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1>);
                    
                    if (bytes_transferred == expected_size)
                    {
                        // 数据正确，解析并入队
                        lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1> frame;
                        std::memcpy(&frame, imu_recv_buffer_.data(), expected_size);
                        frame.timestamp = clock_.now().nanoseconds();
                        imu_queue_.try_enqueue(std::move(frame));
                    }
                    // 无论数据是否正确，都继续接收
                    // 使用post异步调度下一次接收，避免在回调中直接调用导致栈溢出和CPU过载
                    boost::asio::post(imu_socket_->get_executor(), 
                        [this]() { startReceiveIMU(); });
                });
        }
        void startHeartbeat()
        {
            if (!is_running_.load(std::memory_order_acquire))
            {
                return;
            }
            heartbeat_timer_->expires_after(std::chrono::seconds(1));
            heartbeat_timer_->async_wait([this](const boost::system::error_code &ec)
                                         {
                if (ec || !is_running_.load(std::memory_order_acquire)) {
                    return;
                }
                
                    lidar_base_frame::Frame<lidar_base_frame::HeartBeat> heartbeat_frame;
                    try {
                        if (sendCommand(frameToSpan(heartbeat_frame))) {
                            // 心跳成功，继续定时
                            RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ]心跳成功", sn_.c_str());
                            startHeartbeat();
                        } else {
                            // 心跳失败，重置所有标志位，启动重连
                            RCLCPP_WARN(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ]心跳失败，准备重连", sn_.c_str());
                            is_receiving_.store(false, std::memory_order_release);
                            is_running_.store(false, std::memory_order_release);
                            state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                            startReconnect();
                        } 
                    }
                    catch(const std::exception& e) {
                        // 异常，重置所有标志位，启动重连
                        RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ]心跳异常: %s，准备重连", sn_.c_str(), e.what());
                        is_receiving_.store(false, std::memory_order_release);
                        is_running_.store(false, std::memory_order_release);
                        state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                        startReconnect();
                    }
                    catch(...) {
                        // 异常，重置所有标志位，启动重连
                        RCLCPP_ERROR(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ]心跳发生未知异常，准备重连", sn_.c_str());
                        is_receiving_.store(false, std::memory_order_release);
                        is_running_.store(false, std::memory_order_release);
                        state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                        startReconnect();
                    } });
        }
        void startReconnect()
        {
            reconnect_timer_->expires_after(reconnect_interval_);
            reconnect_timer_->async_wait([this](const boost::system::error_code &ec)
                                         {
                if (ec || is_running_.load(std::memory_order_acquire)) {
                    return;
                }
                try {
                    closeSockets();
                    openSockets();
                    // 重新启动ACK接收（重连时必须重新启动）
                    startReceiveACK();
                    if (connect()) {
                        return; // 重连成功，停止定时器
                    }
                } catch (...) {
                    // 重连失败，继续尝试
                }
                startReconnect(); });
        }

        /**
         * @brief 打开所有socket
         */
        void openSockets()
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
        void closeSockets()
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
            cmd_socket_ = std::make_unique<boost::asio::ip::udp::socket>(cpi_io_context_);
            imu_socket_ = std::make_unique<boost::asio::ip::udp::socket>(cpi_io_context_);
            pointcloud_socket_ = std::make_unique<boost::asio::ip::udp::socket>(cpi_io_context_);
            heartbeat_timer_ = std::make_unique<boost::asio::steady_timer>(hr_io_context_);
            reconnect_timer_ = std::make_unique<boost::asio::steady_timer>(hr_io_context_);
            sender_endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(lidar_ip_), 65000);
            auto ports = network_tools::getAvailablePorts(3);
            std::tie(cmd_port_, imu_port_, pointcloud_port_) = std::make_tuple(ports[0], ports[1], ports[2]);
            // std::cout << "为雷达 " << sn_ << " 分配端口: cmd=" << static_cast<int>(cmd_port_)
            //           << ", imu=" << static_cast<int>(imu_port_) << ", pointcloud=" << static_cast<int>(pointcloud_port_)
            //           << std::endl;

            io_threads_.emplace_back([this]()
                                     { hr_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { cpi_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { cpi_io_context_.run(); });
            RCLCPP_DEBUG(rclcpp::get_logger("lidar_base"), "雷达设备[ %s ] 分配端口: cmd=%u, imu=%u, pointcloud=%u",
                         sn_.c_str(), cmd_port_, imu_port_, pointcloud_port_);
        }

    protected:
        std::string sn_;

    private:
        std::string lidar_ip_;
        std::string local_ip_;

        uint16_t cmd_port_;        // 命令端口
        uint16_t pointcloud_port_; // 数据端口
        uint16_t imu_port_;        // IMU端口

        std::mutex send_mutex_;
        boost::asio::io_context hr_io_context_, cpi_io_context_;
        boost::asio::strand<boost::asio::io_context::executor_type> cmd_strand_;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> hr_work_guard_, cpi_work_guard_;

        boost::asio::ip::udp::endpoint remote_endpoint_;                  // 远程端点
        boost::asio::ip::udp::endpoint sender_endpoint_;                  // 发送端点
        std::unique_ptr<boost::asio::ip::udp::socket> cmd_socket_;        // 命令socket
        std::unique_ptr<boost::asio::ip::udp::socket> pointcloud_socket_; // 点云socket
        std::unique_ptr<boost::asio::ip::udp::socket> imu_socket_;        // IMU socket
        std::unique_ptr<boost::asio::steady_timer> heartbeat_timer_;      // 心跳定时器
        std::unique_ptr<boost::asio::steady_timer> reconnect_timer_;      // 重连定时器

        std::vector<std::thread> io_threads_; // asio工作线程

        std::vector<uint8_t> ack_recv_buffer_;        // 用于存储接收到的ACK数据
        std::vector<uint8_t> pointcloud_recv_buffer_; // 用于存储接收到的点云数据
        std::vector<uint8_t> imu_recv_buffer_;        // 用于存储接收到的IMU数据

        moodycamel::BlockingConcurrentQueue<lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96>> pointcloud_queue_;
        moodycamel::BlockingConcurrentQueue<lidar_base_frame::DataFrame<lidar_base_frame::ImuData, 1>> imu_queue_;
        moodycamel::BlockingConcurrentQueue<std::pair<uint16_t, bool>> ack_queue_;

        std::vector<lidar_base_frame::DataFrame<lidar_base_frame::SingleEchoRectangularData, 96>> batch_buffer_; // 用于批量接收点云数据的临时缓冲区

        rclcpp::Clock clock_;

        // 状态
        std::atomic<LidarState> state_;
        std::atomic<bool> is_running_{false};
        std::atomic<bool> is_receiving_{false};

        // 重连时间间隔
        static constexpr std::chrono::seconds reconnect_interval_{20};
        // cmd超时时间
        static constexpr std::chrono::seconds cmd_timeout_{2};
    };

} // namespace lidar_base
