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
    using namespace frame_tools;
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
    class LidarBase : public rclcpp::Node
    {

    public:
        /**
         * @brief 构造函数
         */
        LidarBase(const std::string lidar_ip, const std::string local_ip, const std::string sn)
            : rclcpp::Node("lidar_node_" + sn),
              lidar_ip_(std::move(lidar_ip)), local_ip_(std::move(local_ip)), sn_(std::move(sn)),
              hc_work_guard_(boost::asio::make_work_guard(hc_io_context_)),
              pi_work_guard_(boost::asio::make_work_guard(pi_io_context_)),
              work_guard_(boost::asio::make_work_guard(work_context_)),
              ack_recv_buffer_(512), pointcloud_recv_buffer_(2048), imu_recv_buffer_(1024),
              state_(LidarState::DISCONNECTED), is_running_(false), is_receiving_(false)
        {
            init();
            openSockets();
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

            // 3. 关闭所有socket，让阻塞的receive()立即返回
            closeSockets();

            // 4. 释放work_guard，让io_context可以退出
            hc_work_guard_.reset();
            pi_work_guard_.reset();
            work_guard_.reset();

            // 5. 停止io_context
            hc_io_context_.stop();
            pi_io_context_.stop();
            work_context_.stop();

            // 6. 等待所有线程结束
            for (auto &thread : io_threads_)
            {
                if (thread.joinable())
                {
                    thread.join();
                }
            }

            // 7. 清空队列
            std::pair<double, base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96>> pc_item;
            while (pointcloud_queue_.try_dequeue(pc_item))
                ;
            std::pair<double, base_frame::DataFrame<base_frame::ImuData, 1>> imu_item;
            while (imu_queue_.try_dequeue(imu_item))
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
         * @brief 获取imu队列索引
         */
        moodycamel::BlockingConcurrentQueue<std::pair<double, base_frame::DataFrame<base_frame::ImuData, 1>>> &getIMUQueue()
        {
            return imu_queue_;
        }
        /**
         * @brief 获取点云队列索引
         */
        moodycamel::BlockingConcurrentQueue<std::pair<double, base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96>>> &getPointcloudQueue()
        {
            return pointcloud_queue_;
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
            base_frame::Frame<base_frame::SetLaserStatus> frame(0x01);
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
            base_frame::Frame<base_frame::SetLaserStatus> frame(0x00);
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
            base_frame::Frame<base_frame::SetIMUFrequency> frame(0x01);
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
            base_frame::Frame<base_frame::SetIMUFrequency> frame(0x00);
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
    protected:
        /**
         * @brief 采集数据接口（由子类实现具体逻辑）
         */
        virtual void collect() = 0;

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
                RCLCPP_DEBUG(this->get_logger(), "雷达设备[ %s ] 发送命令: %s", sn_.c_str(), GET_ACK_NAME(data).data());
            }
            catch (const boost::system::system_error &e)
            {

                throw std::runtime_error("发送命令失败: " + std::string(e.what()));
            }
            try
            {
                cmd_socket_->receive(boost::asio::buffer(ack_recv_buffer_.data(), ack_recv_buffer_.capacity()));
                RCLCPP_DEBUG(this->get_logger(), "雷达设备[ %s ] 接收ACK: %s", sn_.c_str(), GET_ACK_NAME(ack_recv_buffer_).data());
                return GET_ACK_RET_CODE(ack_recv_buffer_) == 0x00;
            }
            catch (const boost::system::system_error &e)
            {
                throw std::runtime_error("接收ACK失败: " + std::string(e.what()));
            }
        }

        void startReceive()
        {
            if (is_receiving_.load(std::memory_order_acquire))
            {
                return;
            }
            is_receiving_.store(true, std::memory_order_release);
            state_.store(LidarState::COLLECTING, std::memory_order_release);
            startReceivePointcloud();
            startReceiveIMU();
        }
        /**
         * @brief 停止数据接收
         */
        void stopReceive()
        {
            if (!is_receiving_.load(std::memory_order_acquire))
            {
                return;
            }
            is_receiving_.store(false, std::memory_order_release);

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
            std::pair<double, base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96>> pc_item;
            while (pointcloud_queue_.try_dequeue(pc_item))
                ;
            std::pair<double, base_frame::DataFrame<base_frame::ImuData, 1>> imu_item;
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
            if (is_running_.load(std::memory_order_acquire))
            {
                return true;
            }
            base_frame::Frame<base_frame::HandShake> handshake_frame(local_ip_, pointcloud_port_, cmd_port_, imu_port_);
            if (sendCommand(frameToSpan(handshake_frame)))
            {
                is_running_.store(true, std::memory_order_release);
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
            if (!is_running_.load(std::memory_order_acquire))
            {
                return true;
            }

            // 先停止接收
            if (is_receiving_.load(std::memory_order_acquire))
            {
                stopReceive();
            }

            base_frame::Frame<base_frame::Disconnect> disconnect_frame;
            if (sendCommand(frameToSpan(disconnect_frame)))
            {
                is_running_.store(false, std::memory_order_release);
                is_receiving_.store(false, std::memory_order_release);
                state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                return true;
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
                [this](const boost::system::error_code & /*ec*/, std::size_t bytes_transferred)
                {
                    size_t expected_size = sizeof(base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96>);
                    if (!is_receiving_.load(std::memory_order_acquire))
                    {
                        return;
                    }
                    if (bytes_transferred != expected_size)
                    {
                        // 数据包大小不匹配，丢弃并继续接收
                        startReceivePointcloud();
                        return;
                    }
                    // 解析点云数据包
                    base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96> frame;
                    std::memcpy(&frame, pointcloud_recv_buffer_.data(), expected_size);
                    pointcloud_queue_.try_enqueue(std::make_pair(this->now().seconds(), std::move(frame)));
                    // 继续接收下一个数据包
                    startReceivePointcloud();
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
                [this](const boost::system::error_code & /*ec*/, std::size_t bytes_transferred)
                {
                    size_t expected_size = sizeof(base_frame::DataFrame<base_frame::ImuData, 1>);
                    if (!is_receiving_.load(std::memory_order_acquire))
                    {
                        return;
                    }
                    if (bytes_transferred != expected_size)
                    {
                        // 数据包大小不匹配，丢弃并继续接收
                        startReceiveIMU();
                        return;
                    }
                    // 解析IMU数据包
                    base_frame::DataFrame<base_frame::ImuData, 1> frame;
                    std::memcpy(&frame, imu_recv_buffer_.data(), expected_size);
                    imu_queue_.try_enqueue(std::make_pair(this->now().seconds(), std::move(frame)));
                    // 继续接收下一个数据包
                    startReceiveIMU();
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
                
                    if (!is_running_.load(std::memory_order_acquire)) {
                        return;
                    }
                    base_frame::Frame<base_frame::HeartBeat> heartbeat_frame;
                    try {
                        if (sendCommand(frameToSpan(heartbeat_frame))) {
                            // 心跳成功，继续定时
                            RCLCPP_DEBUG(this->get_logger(), "雷达设备[ %s ]心跳成功", sn_.c_str());
                            startHeartbeat();
                        } else {
                            // 心跳失败，重置所有标志位，启动重连
                            RCLCPP_WARN(this->get_logger(), "雷达设备[ %s ]心跳失败，准备重连", sn_.c_str());
                            is_receiving_.store(false, std::memory_order_release);
                            is_running_.store(false, std::memory_order_release);
                            state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                            startReconnect();
                        } 
                    }
                    catch(const std::exception& e) {
                        // 异常，重置所有标志位，启动重连
                        RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ]心跳异常: %s，准备重连", sn_.c_str(), e.what());
                        is_receiving_.store(false, std::memory_order_release);
                        is_running_.store(false, std::memory_order_release);
                        state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                        startReconnect();
                    }
                    catch(...) {
                        // 异常，重置所有标志位，启动重连
                        RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ]心跳发生未知异常，准备重连", sn_.c_str());
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
            cmd_socket_ = std::make_unique<boost::asio::ip::udp::socket>(hc_io_context_);
            imu_socket_ = std::make_unique<boost::asio::ip::udp::socket>(pi_io_context_);
            pointcloud_socket_ = std::make_unique<boost::asio::ip::udp::socket>(pi_io_context_);
            heartbeat_timer_ = std::make_unique<boost::asio::steady_timer>(hc_io_context_);
            reconnect_timer_ = std::make_unique<boost::asio::steady_timer>(hc_io_context_);
            sender_endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(lidar_ip_), 65000);
            auto ports = network_tools::getAvailablePorts(3);
            std::tie(cmd_port_, imu_port_, pointcloud_port_) = std::make_tuple(ports[0], ports[1], ports[2]);
            std::cout << "为雷达 " << sn_ << " 分配端口: cmd=" << static_cast<int>(cmd_port_)
                      << ", imu=" << static_cast<int>(imu_port_) << ", pointcloud=" << static_cast<int>(pointcloud_port_)
                      << std::endl;
            pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_" + sn_, 10);
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_" + sn_, 10);
            collect_service_ = this->create_service<std_srvs::srv::Trigger>("lidar_collect_" + sn_,
                                                                            std::bind(&LidarBase::handleTrigger, this, std::placeholders::_1, std::placeholders::_2));

            io_threads_.emplace_back([this]()
                                     { hc_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { hc_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { pi_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { pi_io_context_.run(); });
            io_threads_.emplace_back([this]()
                                     { work_context_.run(); });
        }

        void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            std::lock_guard<std::mutex> lock(service_mutex_);
            (void)request;
            RCLCPP_INFO(this->get_logger(), "接收到采集请求");
            if (is_receiving_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "当前正在采集数据，拒绝新的采集请求");
                response->success = false;
                response->message = "当前正在采集数据，拒绝新的采集请求";
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "当前未采集数据，准备开始采集");
                response->success = true;
                response->message = "采集命令已接受，正在启动采集";
                boost::asio::post(work_context_, std::bind(&LidarBase::collect, this));
                return;
            }
        }

        // 基础成员
        std::string lidar_ip_;
        std::string local_ip_;
        std::string sn_;

        uint16_t cmd_port_;        // 命令端口
        uint16_t pointcloud_port_; // 数据端口
        uint16_t imu_port_;        // IMU端口

        std::mutex send_mutex_, pointcloud_mutex_, service_mutex_;
        std::condition_variable pointcloud_cv_;
        boost::asio::io_context hc_io_context_, pi_io_context_, work_context_;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> hc_work_guard_, pi_work_guard_, work_guard_;

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

        moodycamel::BlockingConcurrentQueue<std::pair<double, base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96>>> pointcloud_queue_;
        moodycamel::BlockingConcurrentQueue<std::pair<double, base_frame::DataFrame<base_frame::ImuData, 1>>> imu_queue_;

        // 状态
        std::atomic<LidarState> state_;
        std::atomic<bool> is_running_{false};
        std::atomic<bool> is_receiving_{false};

        // 重连时间间隔
        static constexpr std::chrono::seconds reconnect_interval_{20};
        // cmd超时时间
        static constexpr std::chrono::seconds cmd_timeout_{2};

    protected:
        // 发布者
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        // 采集服务
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr collect_service_;
    };

} // namespace lidar_base
