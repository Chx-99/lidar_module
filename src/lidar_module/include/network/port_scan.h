// Port allocator - header-only, Boost-based, mutex-protected implementation
#pragma once

#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <net/if.h>
#include <netinet/in.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <ifaddrs.h>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

// #define AUTO_PORT_MANAGER

namespace network_tools {

// 获取所有本地网卡的IPv4地址
inline std::vector<std::tuple<std::string, std::string>> getAllLocalInterfaces() {
    std::vector<std::tuple<std::string, std::string>> interfaces;  // <接口名, IP地址>

    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        std::cerr << "获取网络接口失败" << std::endl;
        return interfaces;
    }

    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        // 跳过无效地址、非IPv4地址
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) { continue; }

        sockaddr_in* addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(addr->sin_addr), ip_str, INET_ADDRSTRLEN);

        std::string ip(ip_str);
        std::string iface_name(ifa->ifa_name);

        // 跳过回环地址
        if (ip.find("127.") == 0) { continue; }

        interfaces.emplace_back(iface_name, ip);
    }

    freeifaddrs(ifaddr);
    return interfaces;
}


// 根据IP地址查找对应的网络接口名称
inline std::string getInterfaceNameFromIp(const std::string& ip_address) {
    // 预先将IP地址字符串转换为二进制
    struct in_addr target_addr;
    if (inet_pton(AF_INET, ip_address.c_str(), &target_addr) != 1) {
        throw std::runtime_error("无效的IP地址格式: " + ip_address);
    }

    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) { throw std::runtime_error("获取网络接口失败"); }

    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        // 跳过无效地址和非IPv4地址
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) { continue; }

        sockaddr_in* addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        // 直接比较二进制地址（32位整数比较），避免字符串转换和比较
        if (addr->sin_addr.s_addr == target_addr.s_addr) {
            std::string interface_name(ifa->ifa_name);
            freeifaddrs(ifaddr);
            return interface_name;
        }
    }

    freeifaddrs(ifaddr);
    throw std::runtime_error("未找到与IP匹配的网络接口: " + ip_address);
}

// 端口扫描函数
inline std::vector<std::uint16_t> getAvailablePorts(std::size_t n = 1, std::uint16_t min_port = 48000,
                                                    std::uint16_t max_port = 54000) {
    if (n == 0) return {};
    if (min_port >= max_port) { throw std::invalid_argument("最小端口号必须小于最大端口号"); }

    const std::size_t port_range = max_port - min_port + 1;
    if (n > port_range) {
        throw std::invalid_argument("请求的端口数量 (" + std::to_string(n) + ") 超过可用范围 (" +
                                    std::to_string(port_range) + ")");
    }

    std::vector<std::uint16_t> available_ports;
    available_ports.reserve(n);

    // 创建临时的 io_context 用于端口检测
    boost::asio::io_context io_ctx;

    // 使用多个熵源来初始化随机数生成器，确保真正的随机性
    std::random_device rd;
    std::mt19937 rng(rd());

    std::uniform_int_distribution<std::uint32_t> dist(min_port, max_port);
    std::unordered_set<std::uint16_t> tried;     // 使用 unordered_set 提升查找性能
    tried.reserve(std::min(n * 2, port_range));  // 预分配空间

    // Lambda 函数：测试端口是否可用
    auto test_port = [&io_ctx](std::uint16_t port) -> bool {
        try {
            boost::asio::ip::tcp::acceptor acceptor(io_ctx);
            boost::asio::ip::tcp::endpoint ep(boost::asio::ip::tcp::v4(), port);
            acceptor.open(ep.protocol());
            acceptor.set_option(boost::asio::socket_base::reuse_address(true));
            acceptor.bind(ep);
            acceptor.close();
            return true;
        } catch (const std::exception&) { return false; }
    };

    // 先进行随机尝试，这样在端口较多可用时能快速找到
    const std::size_t max_random_tries = std::min(port_range, n * 3);  // 限制随机尝试次数
    while (available_ports.size() < n && tried.size() < max_random_tries) {
        std::uint16_t port = static_cast<std::uint16_t>(dist(rng));

        // 跳过已经尝试过的端口
        if (!tried.insert(port).second) continue;

        if (test_port(port)) { available_ports.push_back(port); }
    }

    // 如果随机尝试不够，进行线性扫描
    if (available_ports.size() < n) {
        for (std::uint16_t port = min_port; port <= max_port && available_ports.size() < n; ++port) {
            // 已经尝试过的跳过
            if (tried.count(port) > 0) continue;

            if (test_port(port)) { available_ports.push_back(port); }
        }
    }

    // 如果找不到足够的端口，根据参数决定是否抛出异常
    if (available_ports.size() < n) {
        throw std::runtime_error("无法找到足够的可用端口: 需要 " + std::to_string(n) + " 个，只找到 " +
                                 std::to_string(available_ports.size()) + " 个 (范围: " + std::to_string(min_port) +
                                 "-" + std::to_string(max_port) + ")");
    }

    return available_ports;
}

};  // namespace network_tools
