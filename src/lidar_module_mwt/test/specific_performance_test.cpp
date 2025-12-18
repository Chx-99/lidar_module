#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>

// 包含两个版本的数据结构头文件
#include "base/dataStruct.h"   // 旧版本
#include "base/data_struct.h"  // 新版本
#include "base/tools.h"

using namespace std::chrono;

// 定义时间测量宏
#define MEASURE_TIME(operation, iterations, name)                                                               \
    auto start_##name = high_resolution_clock::now();                                                           \
    for (int i = 0; i < iterations; ++i) { operation; }                                                         \
    auto end_##name = high_resolution_clock::now();                                                             \
    auto duration_##name = duration_cast<nanoseconds>(end_##name - start_##name);                               \
    std::cout << #name << " (" << iterations << " iterations) 执行时间: " << duration_##name.count() << " 纳秒" \
              << std::endl;                                                                                     \
    std::cout << "平均每次执行时间: " << duration_##name.count() / iterations << " 纳秒" << std::endl;

// 1. 输出uint8_t数组的二进制表示
void printUint8ArrayBinary(const uint8_t* arr, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        // 输出每个字节的8位二进制表示
        std::cout << std::bitset<8>(arr[i]) << " ";
    }
    std::cout << std::endl;
}

// 2. 输出vector<uint8_t>的二进制表示
void printVectorBinary(const std::vector<uint8_t>& vec) {
    for (uint8_t byte : vec) { std::cout << std::bitset<8>(byte) << " "; }
    std::cout << std::endl;
}

using namespace base_frame;
using namespace frame_tools;

int main() {
    std::cout << "=== 针对特定函数的性能对比测试 ===" << std::endl;
    std::cout << "重点测试 getAckNameCompileTime 函数和 TAG_MAP 查找性能\n" << std::endl;

    const int iterations = 1000000;  // 一百万次迭代

    // 测试新版本的 getAckNameCompileTime 函数性能 (使用switch-case实现)
    std::cout << ">>> 新版本 getAckNameCompileTime 函数性能测试 <<<" << std::endl;

    MEASURE_TIME(volatile auto result = getAckNameCompileTime(0x00, 0x01);
                 , iterations, New_Version_get_ack_name_compile_time_HandShake)

    MEASURE_TIME(volatile auto result = getAckNameCompileTime(0x00, 0x03);
                 , iterations, New_Version_get_ack_name_compile_time_HeartBeat)

    MEASURE_TIME(volatile auto result = getAckNameCompileTime(0x01, 0x08);
                 , iterations, New_Version_get_ack_name_compile_time_IMU_Frequency)

    // 测试旧版本的 TAG_MAP 查找性能 (使用unordered_map实现)
    std::cout << "\n>>> 旧版本 TAG_MAP 查找性能测试 <<<" << std::endl;

    MEASURE_TIME(volatile auto result = old_version::TAG_MAP[std::make_tuple(0x01, 0x00, 0x01)];
                 , iterations, Old_Version_TAG_MAP_Lookup_HandShake)

    MEASURE_TIME(volatile auto result = old_version::TAG_MAP[std::make_tuple(0x01, 0x00, 0x03)];
                 , iterations, Old_Version_TAG_MAP_Lookup_HeartBeat)

    MEASURE_TIME(volatile auto result = old_version::TAG_MAP[std::make_tuple(0x01, 0x01, 0x08)];
                 , iterations, Old_Version_TAG_MAP_Lookup_IMU_Frequency)

    // 测试结果一致性
    std::cout << "\n>>> 结果一致性验证 <<<" << std::endl;

    std::cout << "新版本 HandShake ACK 名称: " << getAckNameCompileTime(0x00, 0x01) << std::endl;
    std::cout << "旧版本 HandShake ACK 名称: " << old_version::TAG_MAP[std::make_tuple(0x01, 0x00, 0x01)] << std::endl;

    std::cout << "新版本 HeartBeat ACK 名称: " << getAckNameCompileTime(0x00, 0x03) << std::endl;
    std::cout << "旧版本 HeartBeat ACK 名称: " << old_version::TAG_MAP[std::make_tuple(0x01, 0x00, 0x03)] << std::endl;

    std::cout << "新版本 IMU Frequency ACK 名称: " << getAckNameCompileTime(0x01, 0x08) << std::endl;
    std::cout << "旧版本 IMU Frequency ACK 名称: " << old_version::TAG_MAP[std::make_tuple(0x01, 0x01, 0x08)]
              << std::endl;

    // 测试其他相关结构体性能
    std::cout << "\n>>> 其他相关结构体性能测试 <<<" << std::endl;

    Frame<HandShake> frame1("192.168.1.103", 55000, 45000, 65000);
    std::cout << "-----新-----" << std::endl;
    printUint8ArrayBinary(reinterpret_cast<const uint8_t*>(&frame1), frame1.length);
    std::cout << "---------" << std::endl;

    std::string ip1 = "192.168.1.103";
    old_version::Frame<old_version::handShake> frame2(getIpVector(ip1), 55000, 45000, 65000);
    // std::cout << frame2 << std::endl;
    std::cout << "-----旧-----" << std::endl;
    auto data = frame2.serialize();
    printUint8ArrayBinary(data.data(), data.size());
    std::cout << "---------" << std::endl;

    Frame<HeartBeat> heart1;
    std::cout << "-----新-----" << std::endl;
    printUint8ArrayBinary(reinterpret_cast<const uint8_t*>(&heart1), heart1.length);
    std::cout << "---------" << std::endl;

    old_version::Frame<old_version::heartBeat> heart2;
    // std::cout << frame2 << std::endl;
    std::cout << "-----旧-----" << std::endl;
    auto data2 = heart2.serialize();
    printUint8ArrayBinary(data2.data(), data2.size());
    std::cout << "---------" << std::endl;

    MEASURE_TIME(Frame<HandShake> frame("192.168.1.103", 55000, 45000, 65000); FRAME_TO_SPAN(frame);
                 , iterations, New_Version_HandShake_Creation)

    MEASURE_TIME(std::string ip = "192.168.1.103";
                 old_version::Frame<old_version::handShake> frame_o(getIpVector(ip), 55000, 45000, 65000);
                 , iterations, Old_Version_handShake_Creation)

    MEASURE_TIME(Frame<HeartBeat> heart_n; FRAME_TO_SPAN(heart_n);, iterations, New_Version_HeartBeat_Creation)

    MEASURE_TIME(old_version::Frame<old_version::heartBeat> heart_o;, iterations, Old_Version_heartBeat_Creation)

    std::cout << "\n=== 性能测试完成 ===" << std::endl;

    return 0;
}