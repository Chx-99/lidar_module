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

namespace lidar_module {
using namespace base_frame;
using namespace free_queue;

class Lidar {
public:
    Lidar(boost::asio::io_context& io_context, std::string lidar_ip, std::string local_ip, std::string sn)
        : lidar_ip_(std::move(lidar_ip)),
          local_ip_(std::move(local_ip)),
          sn_(std::move(sn)),
          io_context_(io_context),
          cmd_socket_(io_context),
          imu_socket_(io_context),
          pointcloud_socket_(io_context),
          heartbeat_timer_(io_context),
          ack_producer_token_(SharedQueues::instance().ackQueue()),
          pointcloud_producer_token_(SharedQueues::instance().pointCloudQueue()),
          imu_producer_token_(SharedQueues::instance().imuQueue()) {
        // 分配端口
        try {
            auto ports = network_tools::getAvailablePorts(3);
            std::tie(cmd_port_, imu_port_, pointcloud_port_) = std::make_tuple(ports[0], ports[1], ports[2]);

            // 绑定socket到对应端口 - cmd_socket绑定到本地IP和端口用于发送命令
            cmd_socket_.open(boost::asio::ip::udp::v4());
            cmd_socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(local_ip_), cmd_port_));

            // imu和pointcloud socket只需要绑定到端口即可（接收数据）
            imu_socket_.open(boost::asio::ip::udp::v4());
            imu_socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), imu_port_));

            pointcloud_socket_.open(boost::asio::ip::udp::v4());
            pointcloud_socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), pointcloud_port_));

            remote_endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::make_address_v4(lidar_ip_), 65000);

            cmd_socket_.set_option(boost::asio::socket_base::broadcast(true));
            imu_socket_.set_option(boost::asio::socket_base::reuse_address(true));
            pointcloud_socket_.set_option(boost::asio::socket_base::reuse_address(true));

            auto interface_name = network_tools::getInterfaceNameFromIp(local_ip_);

            setsockopt(cmd_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
                       interface_name.size());
            setsockopt(imu_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
                       interface_name.size());
            setsockopt(pointcloud_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, interface_name.c_str(),
                       interface_name.size());

            // ACK包较小，保留临时缓冲区用于同步等待机制
            ack_recv_buffer_.reserve(1024);

            // 注意：不在构造函数中调用connect()
            // 需要在io_context.run()启动后再调用connect()
        } catch (const boost::system::system_error& e) {
            std::cerr << "雷达 " << sn_ << " 初始化失败: " << e.what() << std::endl;
            std::cerr << "详细信息: 本地IP=" << local_ip_ << ", 雷达IP=" << lidar_ip_ << ", cmd端口=" << cmd_port_
                      << ", imu端口=" << imu_port_ << ", 点云端口=" << pointcloud_port_ << std::endl;
            throw;
        } catch (const std::exception& e) {
            std::cerr << "雷达 " << sn_ << " 初始化失败: " << e.what() << std::endl;
            throw;
        }
    }
    ~Lidar() { disconnect(); }

    // 连接雷达
    void connect() {
        // 启动异步接收（在io_context线程中执行）
        startReceiveAck();

        // 1. 发送握手指令并等待ACK
        base_frame::Frame<base_frame::HandShake> handshake_frame(local_ip_, pointcloud_port_, cmd_port_, imu_port_);
        sendCommandAndWaitAck(FRAME_TO_SPAN(handshake_frame), "握手");

        // 2. 发送开启激光指令并等待ACK
        base_frame::Frame<base_frame::SetLaserStatus> laser_frame(0x01);
        sendCommandAndWaitAck(FRAME_TO_SPAN(laser_frame), "开启激光");

        // 3. 发送设置IMU频率指令并等待ACK
        base_frame::Frame<base_frame::SetIMUFrequency> imu_frame(0x01);
        sendCommandAndWaitAck(FRAME_TO_SPAN(imu_frame), "设置IMU频率");

        // 初始化成功，标记为运行状态
        is_running_.store(true, std::memory_order_release);

        // 启动其他接收和心跳
        startReceivePointCloud();
        startReceiveIMU();
        startHeartbeat();
    }

    // 断开连接
    void disconnect() {
        is_running_.store(false, std::memory_order_release);
        heartbeat_timer_.cancel();
        cmd_socket_.close();
        imu_socket_.close();
        pointcloud_socket_.close();
        ack_recv_buffer_.clear();
    }

    // 发送指令帧
    bool sendCommandFrame(std::span<const uint8_t> frame) {
        try {
            cmd_socket_.send_to(boost::asio::buffer(frame.data(), frame.size()), remote_endpoint_);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "雷达 " << sn_ << " 发送命令失败: " << e.what() << std::endl;
            return false;
        }
    }

private:
    // 发送命令并同步等待ACK（用于初始化阶段）
    void sendCommandAndWaitAck(std::span<const uint8_t> frame, const std::string& step_name) {
        // 准备promise用于同步等待ACK
        auto ack_promise = std::make_shared<std::promise<bool>>();
        auto ack_future = ack_promise->get_future();

        // 设置初始化ACK回调
        {
            std::lock_guard<std::mutex> lock(init_mutex_);
            init_ack_callback_ = [ack_promise](bool success) { ack_promise->set_value(success); };
        }

        // 发送命令
        sendCommandFrame(frame);

        // 同步等待ACK
        auto status = ack_future.wait_for(std::chrono::seconds(3));

        if (status == std::future_status::timeout) {
            std::lock_guard<std::mutex> lock(init_mutex_);
            init_ack_callback_ = nullptr;
            throw std::runtime_error("雷达 " + sn_ + " " + step_name + " 超时：未收到ACK");
        }

        if (!ack_future.get()) { throw std::runtime_error("雷达 " + sn_ + " " + step_name + " 失败：retcode非0"); }
    }

    // 心跳定时器回调
    void heartbeatTimerCallback() {
        if (!is_running_.load(std::memory_order_acquire)) { return; }

        // 发送心跳
        base_frame::Frame<base_frame::HeartBeat> heartbeat_frame;
        sendCommandFrame(FRAME_TO_SPAN(heartbeat_frame));

        // 继续下一次心跳
        heartbeat_timer_.expires_after(std::chrono::seconds(1));
        heartbeat_timer_.async_wait([this](const boost::system::error_code& ec) {
            if (!ec) { heartbeatTimerCallback(); }
        });
    }

    void startHeartbeat() {
        heartbeat_timer_.expires_after(std::chrono::seconds(1));
        heartbeat_timer_.async_wait([this](const boost::system::error_code& ec) {
            if (!ec) { heartbeatTimerCallback(); }
        });
    }

    // ACK接收（初始化同步等待 / 运行时队列分发）
    void startReceiveAck() {
        // 创建动态缓冲区适配器
        auto dynamicBuf = boost::asio::dynamic_buffer(ack_recv_buffer_);
        // 为本次接收预留空间
        auto mutableBuffer = dynamicBuf.prepare(256);

        cmd_socket_.async_receive_from(mutableBuffer, sender_endpoint_,
                                       [this](boost::system::error_code ec, std::size_t bytes_received) {
                                           if (!ec && bytes_received > 0) {
                                               auto dynamicBuf = boost::asio::dynamic_buffer(ack_recv_buffer_);
                                               // 提交接收到的数据
                                               dynamicBuf.commit(bytes_received);
                                               // 处理接收到的数据
                                               handleAckReceive(bytes_received);
                                               // 清空缓冲区
                                               dynamicBuf.consume(dynamicBuf.size());
                                           }
                                           // 继续接收（只要socket未关闭）
                                           if (cmd_socket_.is_open()) { startReceiveAck(); }
                                       });
    }

    void handleAckReceive(std::size_t bytes_received) {
        try {
            // 初始化阶段：通过callback通知promise
            if (!is_running_.load(std::memory_order_acquire)) {
                uint8_t ret_code = GET_ACK_RET_CODE(ack_recv_buffer_);
                bool success = (ret_code == 0x00);
                {
                    std::lock_guard<std::mutex> lock(init_mutex_);
                    if (init_ack_callback_) {
                        init_ack_callback_(success);
                        init_ack_callback_ = nullptr;
                    }
                }
            }

            // 推送到队列供外部处理
            pushAckToQueue(std::span<const uint8_t>(ack_recv_buffer_.data(), bytes_received));
            return;
        } catch (const std::out_of_range&) {
            std::cerr << "雷达 " << sn_ << " 收到未知ACK" << std::endl;
        } catch (const std::exception& e) { std::cerr << "雷达 " << sn_ << " 处理ACK异常: " << e.what() << std::endl; }
    }

    void pushAckToQueue(std::span<const uint8_t> data) {
        // 立即拷贝数据到队列，避免缓冲区被覆盖导致数据错误
        AckPacket packet{.sn = sn_, .data = std::vector<uint8_t>(data.begin(), data.end())};
        // 使用ProducerToken提高性能
        SharedQueues::instance().ackQueue().enqueue(ack_producer_token_, std::move(packet));
    }

    // 点云接收（直接用内存池buffer接收，零拷贝）
    void startReceivePointCloud() {
        // 从内存池获取buffer，直接用于接收网络数据
        auto buffer = pointcloud_buffer_pool_.acquire();

        // 确保有足够容量（首次或降级分配时）
        if (buffer->capacity() < 2048) { buffer->reserve(2048); }
        buffer->resize(buffer->capacity());  // 使用全部可用空间接收

        pointcloud_socket_.async_receive_from(
            boost::asio::buffer(buffer->data(), buffer->size()), sender_endpoint_,
            [this, buffer](boost::system::error_code ec, std::size_t bytes_received) {
                if (!ec && bytes_received > 0 && is_running_.load(std::memory_order_acquire)) {
                    // 调整buffer到实际接收的大小
                    buffer->resize(bytes_received);

                    // 直接入队，零拷贝传递（shared_ptr）
                    PointCloudPacket packet{.sn = sn_, .data = buffer};
                    SharedQueues::instance().pointCloudQueue().enqueue(pointcloud_producer_token_, std::move(packet));
                }

                if (pointcloud_socket_.is_open()) { startReceivePointCloud(); }
            });
    }

    // IMU接收（直接用内存池buffer接收，零拷贝）
    void startReceiveIMU() {
        // 从内存池获取buffer，直接用于接收网络数据
        auto buffer = imu_buffer_pool_.acquire();

        // 确保有足够容量（首次或降级分配时）
        if (buffer->capacity() < 1024) { buffer->reserve(1024); }
        buffer->resize(buffer->capacity());  // 使用全部可用空间接收

        imu_socket_.async_receive_from(boost::asio::buffer(buffer->data(), buffer->size()), sender_endpoint_,
                                       [this, buffer](boost::system::error_code ec, std::size_t bytes_received) {
                                           if (!ec && bytes_received > 0 &&
                                               is_running_.load(std::memory_order_acquire)) {
                                               // 调整buffer到实际接收的大小
                                               buffer->resize(bytes_received);

                                               // 直接入队，零拷贝传递（shared_ptr）
                                               IMUPacket packet{.sn = sn_, .data = buffer};
                                               SharedQueues::instance().imuQueue().enqueue(imu_producer_token_,
                                                                                           std::move(packet));
                                           }

                                           if (imu_socket_.is_open()) { startReceiveIMU(); }
                                       });
    }

private:
    // 基础数据成员（无依赖）- 必须先声明，因为在构造函数中先被move
    std::string lidar_ip_;  // 雷达ip
    std::string local_ip_;  // 本地ip
    std::string sn_;        // 雷达序列号

    // io_context 引用（被其他成员依赖，需要先声明）
    boost::asio::io_context& io_context_;  // ASIO上下文 所有雷达公用一个io_context

    // 端口句柄（RAII 管理，自动释放）
    uint8_t cmd_port_;         // 命令端口
    uint8_t pointcloud_port_;  // 数据端口
    uint8_t imu_port_;         // IMU端口

    // 网络组件（依赖 io_context 和端口）
    boost::asio::ip::udp::socket cmd_socket_;         // 用于接收与发送命令
    boost::asio::ip::udp::socket imu_socket_;         // 用于接收IMU数据
    boost::asio::ip::udp::socket pointcloud_socket_;  // 用于接收点云数据
    boost::asio::ip::udp::endpoint remote_endpoint_;  // 远程端点

    // 定时器（依赖 io_context）
    boost::asio::steady_timer heartbeat_timer_;  // 心跳定时器

    // 接收缓冲区（仅ACK需要临时缓冲区，用于同步等待机制）
    std::vector<uint8_t> ack_recv_buffer_;  // 用于存储接收到的ACK数据

    // 发送方端点（用于接收数据时记录来源）
    boost::asio::ip::udp::endpoint sender_endpoint_;

    // 初始化同步机制
    std::mutex init_mutex_;
    std::function<void(bool)> init_ack_callback_;

    // 无锁队列生产者令牌（提高性能）
    moodycamel::ProducerToken ack_producer_token_;
    moodycamel::ProducerToken pointcloud_producer_token_;
    moodycamel::ProducerToken imu_producer_token_;

    // 内存池（零拷贝传递，避免频繁堆分配）
    memory_pool::BufferPool<uint8_t, 32> pointcloud_buffer_pool_;  // 点云数据内存池（大包，高频）
    memory_pool::BufferPool<uint8_t, 16> imu_buffer_pool_;         // IMU数据内存池（中等频率）


    // 运行状态
    std::atomic<bool> is_running_{false};
};
};  // namespace lidar_module