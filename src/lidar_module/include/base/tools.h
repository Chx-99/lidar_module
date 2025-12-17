//
// Created by mwt on 25-1-21.
//

#ifndef NETTOOLS_H
#define NETTOOLS_H

#include <bitset>
#include <ctime>
#include <cstring>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <random>
#include <bits/random.h>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>

#ifdef _WIN32
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#else
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

// Helper function to find the network interface name from an IP address
inline std::string get_interface_name_from_ip(const std::string& ip_address) {
    #ifdef _WIN32
    // Windows implementation using GetAdaptersAddresses
    ULONG buffer_size = 15000;
    std::vector<unsigned char> buffer(buffer_size);
    PIP_ADAPTER_ADDRESSES adapter_addresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.data());

    if (GetAdaptersAddresses(AF_INET, 0, nullptr, adapter_addresses, &buffer_size) == ERROR_BUFFER_OVERFLOW) {
        throw std::runtime_error("Buffer size too small for adapter addresses.");
    }

    for (auto* adapter = adapter_addresses; adapter != nullptr; adapter = adapter->Next) {
        for (auto* unicast = adapter->FirstUnicastAddress; unicast != nullptr; unicast = unicast->Next) {
            sockaddr_in* addr = reinterpret_cast<sockaddr_in*>(unicast->Address.lpSockaddr);
            if (addr->sin_addr.s_addr == inet_addr(ip_address.c_str())) {
                return adapter->AdapterName;
            }
        }
    }

    throw std::runtime_error("No matching network interface found for the given IP address.");
    #else
    // Linux implementation using getifaddrs
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        throw std::runtime_error("Failed to get network interfaces.");
    }

    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }

        sockaddr_in* addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        if (inet_ntoa(addr->sin_addr) == ip_address) {
            std::string interface_name(ifa->ifa_name);
            freeifaddrs(ifaddr);
            return interface_name;
        }
    }

    freeifaddrs(ifaddr);
    throw std::runtime_error("No matching network interface found for the given IP address.");
    #endif
}
/**
 *
 * @param io_context_ io_context_
 * @return 随机可用端口
 */
inline std::vector<unsigned short> getRandomAvailablePort(int size) {
    std::vector<unsigned short> ports;
    using namespace boost::asio;
    boost::asio::io_context io_context_; // 创建一个 io_context_ 对象
    const unsigned short min_port = 50001;
    const unsigned short max_port = 65535;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned short> dis(min_port, max_port);
    for (int i = 0; i < 100; ++i) {
        unsigned short port = dis(gen);
        try {
            // 尝试创建一个 TCP 接受器并绑定到指定端口
            ip::tcp::acceptor acceptor(io_context_);
            ip::tcp::endpoint endpoint(ip::tcp::v4(), port);
            acceptor.open(endpoint.protocol());
            acceptor.bind(endpoint);
            acceptor.close();
            if (std::find(ports.begin(), ports.end(), port) == ports.end()) {
                ports.push_back(port);
            }
            if (ports.size() == size) {
                return ports;
            }
        } catch (std::exception &e) {
            // 绑定失败，说明该端口可能已被占用，继续尝试下一个端口
        }
    }
    throw std::runtime_error("本机暂无可用端口号.");
}

#endif //NETTOOLS_H
