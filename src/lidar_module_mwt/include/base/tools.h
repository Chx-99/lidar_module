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
/**
* @brief 获得该ip的.分割数组
*/
inline std::vector<uint8_t> getIpVector(const std::string &ip) {
    std::vector<uint8_t> ip_vector;
    std::istringstream iss(ip);
    std::string token;

    while (std::getline(iss, token, '.')) {
        int octet = std::stoi(token);
        if (octet < 0 || octet > 255) {
            throw std::runtime_error("Invalid IP octet value: " + token);
        }
        ip_vector.push_back(static_cast<uint8_t>(octet));
    }

    if (ip_vector.size() != 4) {
        throw std::runtime_error("Invalid IP format: " + ip);
    }

    return ip_vector;
}

inline nlohmann::json parseLidarStatusToJson(uint32_t status_code) {
    auto get_bits = [](uint32_t value, int start, int len = 1) -> uint32_t {
        return (value >> start) & ((1 << len) - 1);
    };

    uint32_t temp_status        = get_bits(status_code, 0, 2);   // Bit 0~1
    uint32_t volt_status        = get_bits(status_code, 2, 2);   // Bit 2~3
    uint32_t motor_status       = get_bits(status_code, 4, 2);   // Bit 4~5
    uint32_t dirty_warn         = get_bits(status_code, 6, 2);   // Bit 6~7
    uint32_t firmware_status    = get_bits(status_code, 8);      // Bit 8
    uint32_t pps_status         = get_bits(status_code, 9);      // Bit 9
    uint32_t device_status      = get_bits(status_code, 10);     // Bit 10
    uint32_t fan_status         = get_bits(status_code, 11);     // Bit 11
    uint32_t self_heating       = get_bits(status_code, 12);     // Bit 12
    uint32_t ptp_status         = get_bits(status_code, 13);     // Bit 13
    uint32_t time_sync_status   = get_bits(status_code, 14, 3);  // Bit 14~16
    uint32_t system_status      = get_bits(status_code, 30, 2);  // Bit 30~31

    nlohmann::json j;

    j["raw_bits"] = std::bitset<32>(status_code).to_string();

    j["temp_status"] = {
        {"value", temp_status},
        {"desc", temp_status == 0 ? "正常" :
                  temp_status == 1 ? "偏高/偏低" :
                  "极高/极低"}
    };

    j["volt_status"] = {
        {"value", volt_status},
        {"desc", volt_status == 0 ? "正常" :
                  volt_status == 1 ? "偏高" : "极高"}
    };

    j["motor_status"] = {
        {"value", motor_status},
        {"desc", motor_status == 0 ? "正常" :
                  motor_status == 1 ? "警告" : "错误"}
    };

    j["dirty_warn"] = {
        {"value", dirty_warn},
        {"desc", dirty_warn == 0 ? "无遮挡" : "有遮挡"}
    };

    j["firmware_status"] = {
        {"value", firmware_status},
        {"desc", firmware_status == 0 ? "正常" : "出错，需要升级"}
    };

    j["pps_status"] = {
        {"value", pps_status},
        {"desc", pps_status == 0 ? "无PPS" : "正常"}
    };

    j["device_status"] = {
        {"value", device_status},
        {"desc", device_status == 0 ? "正常" : "寿命警告"}
    };

    j["fan_status"] = {
        {"value", fan_status},
        {"desc", fan_status == 0 ? "正常" : "风扇警告"}
    };

    j["self_heating"] = {
        {"value", self_heating},
        {"desc", self_heating == 0 ? "关闭" : "开启"}
    };

    j["ptp_status"] = {
        {"value", ptp_status},
        {"desc", ptp_status == 0 ? "无1588信号" : "正常"}
    };

    j["time_sync_status"] = {
        {"value", time_sync_status},
        {"desc",
            time_sync_status == 0 ? "未同步" :
            time_sync_status == 1 ? "PTP 1588" :
            time_sync_status == 2 ? "GPS" :
            time_sync_status == 3 ? "PPS" :
            time_sync_status == 4 ? "系统时间异常" : "未知"}
    };

    j["system_status"] = {
        {"value", system_status},
        {"desc",
            system_status == 0 ? "正常" :
            system_status == 1 ? "警告" :
            system_status == 2 ? "错误" : "未知"}
    };

    return j;
}

#endif //NETTOOLS_H
