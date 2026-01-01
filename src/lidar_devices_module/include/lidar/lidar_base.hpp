#pragma once

#include "lidar/base/data_struct.hpp"
#include "lidar/base/port_scan.hpp"
#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace lidar_base
{
    using namespace base_frame;
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
    class LidarBase
    {
    public:
        /**
         * @brief 构造函数
         */
        LidarBase(const std::string &lidar_ip, const std::string &local_ip, const std::string &sn)
            : lidar_ip_(std::move(lidar_ip)), local_ip_(std::move(local_ip)), sn_(std::move(sn)),
        {
        }

        virtual ~LidarBase()
        {
        }

        // 禁止拷贝和移动
        LidarBase(const LidarBase &) = delete;
        LidarBase &operator=(const LidarBase &) = delete;

        /**
         * @brief 开始采集（纯虚函数，由子类实现）
         * @param callback 采集完成回调
         * @return true=成功启动, false=失败
         */
        virtual bool startCollection() = 0;

        /**
         * @brief 获取当前状态
         */
        LidarState getState() const
        {
            return state_.load(std::memory_order_acquire);
        }

        /**
         * @brief 获取雷达SN
         */
        const std::string &getSN() const { return sn_; }

        /**
         * @brief 开启激光
         */
        bool enableLaser()
        {
            Frame<SetLaserStatus> frame(0x01);
            return sendCommand(frameToSpan<SetLaserStatus>(frame));
        }

        /**
         * @brief 关闭激光
         */
        bool disableLaser()
        {
            Frame<SetLaserStatus> frame(0x00);
            return sendCommand(frameToSpan<SetLaserStatus>(frame));
        }

        /**
         * @brief 开启IMU
         */
        bool enableIMU()
        {
            Frame<SetIMUFrequency> frame(0x01);
            return sendCommand(frameToSpan<SetIMUFrequency>(frame));
        }

        /**
         * @brief 关闭IMU
         */
        bool disableIMU()
        {
            Frame<SetIMUFrequency> frame(0x00);
            return sendCommand(frameToSpan<SetIMUFrequency>(frame));
        }

    protected:
        /**
         * @brief 发送命令并等待ACK
         */
        bool sendCommand(std::span<const uint8_t> data)
        {
            std::lock_guard<std::mutex> lock(send_mutex_);
            try
            {
                // TODO: 实现发送逻辑
                return true;
            }
            catch (...)
            {
                return false;
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
            state_.store(LidarState::IDLE, std::memory_order_release);
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
            Frame<HandShake> handshake_frame(local_ip_, pointcloud_port_, cmd_port_, imu_port_);
            if (sendCommand(frameToSpan<HandShake>(handshake_frame)))
            {
                is_running_.store(true, std::memory_order_release);
                state_.store(LidarState::IDLE, std::memory_order_release);
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
            Frame<Disconnect> disconnect_frame;
            if (sendCommand(frameToSpan<Disconnect>(disconnect_frame)))
            {
                is_running_.store(false, std::memory_order_release);
                state_.store(LidarState::DISCONNECTED, std::memory_order_release);
                return true;
            }
            return false;
        }

    private:
        /**
         * @brief 启动数据接收
         */
        void startReceivePointcloud()
        {
            // TODO: 异步接收数据
        }
        void startReceiveIMU()
        {
            // TODO: 异步接收数据
        }
        

        /**
         * @brief 追加点云数据到ROS PointCloud2消息
         * @param cloud ROS PointCloud2消息
         * @param points 点云数据帧
         */
        void appendRosPointcloud(sensor_msgs::msg::PointCloud2 &cloud, const DataFrame<SingleEchoRectangularData, 96> &points)
        {
        }

        void appendRosIMU(sensor_msgs::msg::Imu &imu_msg, const DataFrame<IMUData, 1> &imu_data)
        {
        }

        
        /**
         * @brief 打开所有socket
         */
        bool openSockets()
        {
            auto interface_name = network_tools::getInterfaceNameFromIp(local_ip_);

            // 初始化 cmd_socket
            cmd_socket_->open(boost::asio::ip::udp::v4());
            cmd_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(local_ip_), cmd_port_));
            cmd_socket_->set_option(boost::asio::socket_base::broadcast(true));
            setsockopt(cmd_socket_->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(), interface_name.size());
            struct timeval tv{cmd_timeout_.count(), 0};
            setsockopt(cmd_socket_->native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            // 初始化 imu_socket
            imu_socket_->open(boost::asio::ip::udp::v4());
            imu_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), imu_port_));
            imu_socket_->set_option(boost::asio::socket_base::reuse_address(true));
            setsockopt(imu_socket_->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(), interface_name.size());

            // 初始化 pointcloud_socket
            pointcloud_socket_->open(boost::asio::ip::udp::v4());
            pointcloud_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), pointcloud_port_));
            pointcloud_socket_->set_option(boost::asio::socket_base::reuse_address(true));
            setsockopt(pointcloud_socket_->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(), interface_name.size());
        }

        /**
         * @brief 关闭所有socket
         */
        bool closeSockets()
        {
            boost::system::error_code ec;
            cmd_socket_->close(ec);
            imu_socket_->close(ec);
            pointcloud_socket_->close(ec);
        }

        // 基础成员
        std::string lidar_ip_;
        std::string local_ip_;
        std::string sn_;

        uint16_t cmd_port_;        // 命令端口
        uint16_t pointcloud_port_; // 数据端口
        uint16_t imu_port_;        // IMU端口

        std::mutex send_mutex_;
        boost::asio::io_context hc_io_context_, pi_io_context_;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> hc_work_guard_, pi_work_guard_;

        boost::asio::ip::udp::endpoint remote_endpoint_;                  // 远程端点
        boost::asio::ip::udp::endpoint sender_endpoint_;                  // 发送端点
        std::unique_ptr<boost::asio::ip::udp::socket> cmd_socket_;        // 命令socket
        std::unique_ptr<boost::asio::ip::udp::socket> pointcloud_socket_; // 点云socket
        std::unique_ptr<boost::asio::ip::udp::socket> imu_socket_;        // IMU socket
        std::unique_ptr<boost::asio::steady_timer> heartbeat_timer_;      // 心跳定时器
        std::unique_ptr<boost::asio::steady_timer> reconnect_timer_;      // 重连定时器

        std::vector<std::thread> io_threads_;

        std::vector<uint8_t> ack_recv_buffer_;        // 用于存储接收到的ACK数据
        std::vector<uint8_t> pointcloud_recv_buffer_; // 用于存储接收到的点云数据
        std::vector<uint8_t> imu_recv_buffer_;        // 用于存储接收到的IMU数据

        std::vector<sensor_msgs::msg::PointCloud2> pointcloud_data_buffer_; // 点云数据缓冲区
        std::vector<sensor_msgs::msg::Imu> imu_data_buffer_;                // IMU数据缓冲区

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
