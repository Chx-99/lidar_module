// 雷达扫描器 - header-only 实现，支持多网卡扫描，简洁高效版本
#pragma once

#include "base/data_struct.h"
#include "network/port_scan.h"

#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/udp.hpp>

#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace lidar_module {

// 雷达信息结构体
struct LidarInfo {
    std::string lidar_ip;  // 雷达IP
    std::string local_ip;  // 连接该雷达的本地IP

    LidarInfo() = default;
    LidarInfo(std::string lidar, std::string local) : lidar_ip(std::move(lidar)), local_ip(std::move(local)) {}
};

class LidarScanner {
public:
    /**
     * 构造函数：
     * @param seconds 监听时长（单位：秒）
     * @param port 监听的 UDP 端口（默认55000）
     */
    LidarScanner(int seconds = 5, unsigned short port = 55000) : seconds_(seconds), port_(port) {}

    ~LidarScanner() = default;

    /**
     * 扫描所有网段的雷达（线程安全、简洁版本）
     * @return 雷达信息映射表，键为 SN 码，值为雷达信息（雷达IP和本地IP）
     */
    std::unordered_map<std::string, LidarInfo> searchLidar() {
        // 获取所有本地网卡
        auto interfaces = network_tools::getAllLocalInterfaces();
        if (interfaces.empty()) { throw std::runtime_error("未找到可用的网络接口"); }

        std::cout << "找到 " << interfaces.size() << " 个网络接口:" << std::endl;
        for (const auto& [iface, ip] : interfaces) { std::cout << "  - " << iface << ": " << ip << std::endl; }

        // 创建独立的 io_context
        boost::asio::io_context io_context;

        // 线程安全的结果容器（使用 SN 作为键）
        std::mutex lidars_mutex;
        std::unordered_map<std::string, LidarInfo> lidars;

        // 为每个网卡创建监听socket
        std::vector<std::shared_ptr<boost::asio::ip::udp::socket>> sockets;
        sockets.reserve(interfaces.size());

        for (const auto& [iface_name, local_ip] : interfaces) {
            try {
                auto sock = std::make_shared<boost::asio::ip::udp::socket>(io_context);
                sock->open(boost::asio::ip::udp::v4());
                sock->set_option(boost::asio::socket_base::reuse_address(true));
                sock->set_option(boost::asio::socket_base::broadcast(true));

                // 绑定到指定端口和本地IP
                sock->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(local_ip), port_));

                // 绑定到指定网卡（Linux特定）
                if (setsockopt(sock->native_handle(), SOL_SOCKET, SO_BINDTODEVICE, iface_name.c_str(),
                               iface_name.size()) != 0) {
                    std::cerr << "警告: 无法绑定到网卡 " << iface_name << ": " << strerror(errno) << std::endl;
                }

                std::cout << "成功在 " << iface_name << " (" << local_ip << ") 上监听端口 " << port_ << std::endl;

                // 启动异步接收
                startReceiveOnSocket(sock, local_ip, lidars, lidars_mutex);
                sockets.push_back(sock);

            } catch (const std::exception& e) {
                std::cerr << "无法在 " << iface_name << " (" << local_ip << ") 上创建监听: " << e.what() << std::endl;
            }
        }

        if (sockets.empty()) { throw std::runtime_error("未能在任何网卡上创建监听socket"); }

        // 启动定时器（使用栈变量即可）
        boost::asio::steady_timer timer(io_context);
        timer.expires_after(std::chrono::seconds(seconds_));
        timer.async_wait([&](const boost::system::error_code& ec) {
            if (!ec) {
                std::cout << "扫描超时 (" << seconds_ << " 秒)，停止监听" << std::endl;
                for (auto& sock : sockets) {
                    if (sock && sock->is_open()) {
                        boost::system::error_code ignore_ec;
                        sock->close(ignore_ec);
                    }
                }
            }
        });

        // 直接运行 io_context（阻塞直到完成）
        std::cout << "开始扫描雷达..." << std::endl;
        io_context.run();
        std::cout << "扫描完成，找到 " << lidars.size() << " 个雷达" << std::endl;

        return lidars;
    }

private:
    // 在指定socket上启动异步接收（线程安全版本）
    void startReceiveOnSocket(std::shared_ptr<boost::asio::ip::udp::socket> sock, const std::string& local_ip,
                              std::unordered_map<std::string, LidarInfo>& lidars, std::mutex& lidars_mutex) {
        // 每次接收都创建新的endpoint和buffer（避免竞态条件）
        auto sender_endpoint = std::make_shared<boost::asio::ip::udp::endpoint>();
        auto buffer = std::make_shared<std::vector<uint8_t>>(1024);

        sock->async_receive_from(
            boost::asio::buffer(*buffer), *sender_endpoint,
            [this, sock, local_ip, sender_endpoint, buffer, &lidars,
             &lidars_mutex](const boost::system::error_code& error, std::size_t bytes_transferred) {
                if (error) {
                    if (error == boost::asio::error::operation_aborted) {
                        // 正常关闭
                        return;
                    }
                    std::cerr << "接收错误 (" << local_ip << "): " << error.message() << std::endl;
                    return;
                }

                if (bytes_transferred > 0) {
                    try {
                        if (bytes_transferred >= sizeof(base_frame::Frame<base_frame::BoardCastMSG>)) {
                            auto frame =
                                reinterpret_cast<const base_frame::Frame<base_frame::BoardCastMSG>*>(buffer->data());

                            // 验证帧格式
                            if (frame->sof == 0xAA && frame->data.cmd_set == 0x00 && frame->data.cmd_id == 0x00) {
                                std::string lidar_ip = sender_endpoint->address().to_string();
                                std::string broadcast_code = frame->data.getBroadCastCode();
                                std::string zwkj_sn = frame->data.getzwkjSN();

                                // 线程安全地添加到结果映射表（使用 SN 作为键）
                                {
                                    std::lock_guard<std::mutex> lock(lidars_mutex);
                                    // 使用 emplace 或 insert，如果 SN 已存在则不覆盖
                                    if (lidars.find(zwkj_sn) == lidars.end()) {
                                        // 首次见到这个雷达
                                        std::cout << "收到雷达广播:" << std::endl;
                                        std::cout << "  SN码: " << zwkj_sn << std::endl;
                                        std::cout << "  雷达IP: " << lidar_ip << std::endl;
                                        std::cout << "  本地IP: " << local_ip << std::endl;
                                        std::cout << "  广播码: " << broadcast_code << std::endl;

                                        lidars.emplace(zwkj_sn, LidarInfo(lidar_ip, local_ip));
                                    }
                                }
                            }
                        }
                    } catch (const std::exception& e) { std::cerr << "解析广播消息失败: " << e.what() << std::endl; }
                }

                // 继续接收（如果socket仍然打开）
                if (sock->is_open()) { startReceiveOnSocket(sock, local_ip, lidars, lidars_mutex); }
            });
    }

    int seconds_;
    unsigned short port_;
};

}  // namespace lidar_module