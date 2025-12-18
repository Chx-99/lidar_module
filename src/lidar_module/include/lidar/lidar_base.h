#pragma once

#include "base/data_struct.h"

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

namespace lidar_module
{

    class Lidar
    {
    public:
        Lidar(boost::asio::io_context &io_context, std::string lidar_ip, std::string local_ip, std::string sn, int frequency)
        : io_context_(io_context),
          cmd_socket_(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(local_ip), 0)),
          imu_socket_(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)),
          pointcloud_socket_(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)),
          strand_(boost::asio::make_strand(io_context)),
          lidar_ip_(std::move(lidar_ip)),
          local_ip_(std::move(local_ip)),
          sn_(std::move(sn)),
          frequency_(frequency)
        {
            connect();
        }
        ~Lidar();

        // 连接雷达
        void connect();
        bool appendCommandFrame(std::span<const uint8_t> frame);

    private:
        // 心跳定时器回调
        void heartbeatTimerCallback();

        // 接收指定数据
        void receiveAck();
        void receivePointCloud();
        void receiveIMU();

    private:
        std::string lidar_ip_; // 雷达ip
        std::string local_ip_; // 本地ip
        std::string sn_;       // 雷达序列号
        int frequency_;        // 雷达频率

        uint16_t cmd_port_;       // 命令端口
        uint16_t pointcloud_port_;      // 数据端口
        uint16_t imu_port_;       // IMU端口

        boost::asio::steady_timer heartbeat_timer_; // 心跳定时器

        boost::asio::io_context &io_context_;                                // ASIO上下文 所有雷达公用一个io_context
        boost::asio::ip::udp::socket cmd_socket_;                            // 用于接收与发送命令
        boost::asio::ip::udp::socket imu_socket_;                            // 用于接收IMU数据
        boost::asio::ip::udp::socket pointcloud_socket_;                     // 用于接收点云数据
        boost::asio::ip::udp::endpoint remote_endpoint_;                     // 远程端点
        boost::asio::strand<boost::asio::io_context::executor_type> strand_; // 用于确保异步操作的顺序执行
    }
};