#pragma once

#include "base/data_struct.h"
#include "base/port_scan.h"

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

namespace lidar_module {

class Lidar {
public:
    Lidar(boost::asio::io_context& io_context, std::string lidar_ip, std::string local_ip, std::string sn,
          int frequency) try
        : io_context_(io_context),
          cmd_port_(network_tools::BoostPortAllocator::instance().acquire()),
          pointcloud_port_(network_tools::BoostPortAllocator::instance().acquire()),
          imu_port_(network_tools::BoostPortAllocator::instance().acquire()),
          cmd_socket_(io_context,
                      boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(local_ip), cmd_port_.port())),
          imu_socket_(io_context, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), imu_port_.port())),
          pointcloud_socket_(io_context,
                             boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), pointcloud_port_.port())),
          heartbeat_timer_(io_context),
          strand_(boost::asio::make_strand(io_context)),
          lidar_ip_(std::move(lidar_ip)),
          local_ip_(std::move(local_ip)),
          sn_(std::move(sn)),
          frequency_(frequency) {
        connect();
    } catch (const std::runtime_error& e) {
        // 端口分配失败
        std::cerr << "雷达" << sn << "初始化失败: " << e.what() << std::endl;
        throw;  // 重新抛出或转换为自定义异常
    }
    ~Lidar() {
        // 释放资源
        cmd_socket_.close();
        imu_socket_.close();
        pointcloud_socket_.close();
        heartbeat_timer_.cancel();
    }

    // 连接雷达
    void connect() {}
    bool appendCommandFrame(std::span<const uint8_t> frame) {}

private:
    // 心跳定时器回调
    void heartbeatTimerCallback() {}

    // 接收指定数据
    void receiveAck() {}
    void receivePointCloud() {}
    void receiveIMU() {}

private:
    // 基础数据成员（无依赖）
    std::string lidar_ip_;  // 雷达ip
    std::string local_ip_;  // 本地ip
    std::string sn_;        // 雷达序列号
    int frequency_;         // 雷达频率

    // io_context 引用（被其他成员依赖，需要先声明）
    boost::asio::io_context& io_context_;  // ASIO上下文 所有雷达公用一个io_context

    // 端口句柄（RAII 管理，自动释放）
    network_tools::PortHandle cmd_port_;         // 命令端口
    network_tools::PortHandle pointcloud_port_;  // 数据端口
    network_tools::PortHandle imu_port_;         // IMU端口

    // 网络组件（依赖 io_context 和端口）
    boost::asio::ip::udp::socket cmd_socket_;         // 用于接收与发送命令
    boost::asio::ip::udp::socket imu_socket_;         // 用于接收IMU数据
    boost::asio::ip::udp::socket pointcloud_socket_;  // 用于接收点云数据
    boost::asio::ip::udp::endpoint remote_endpoint_;  // 远程端点

    // 定时器和 strand（依赖 io_context）
    boost::asio::steady_timer heartbeat_timer_;                           // 心跳定时器
    boost::asio::strand<boost::asio::io_context::executor_type> strand_;  // 用于确保异步操作的顺序执行
};

};  // namespace lidar_module