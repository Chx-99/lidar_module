#pragma once
#include "sensor/motor/base/comm_interface.hpp"
#include "sensor/motor/base/plc_device_base.hpp"
#include <string>
#include <iostream>
#include <cstdint>

/**
 * @brief 雷达类型枚举
 */
enum class LidarType
{
    LIDAR_80,  // 80度雷达（Lidar01）
    LIDAR_180, // 180度旋转雷达（LidarR1）
    UNKNOWN    // 未知类型
};

/**
 * @brief 雷达配置信息
 */
struct LidarConfig
{
    // 雷达基本信息
    LidarType type; // 雷达类型
    std::string sn; // 雷达序列号
    std::string lidar_ip;  // 雷达IP地址
    std::string local_ip;  // 本地IP地址

    // 电机配置
    std::string comm_type;   // 通信类型 ("tcp" 或 "rtu" 或 "ttw")
    // 串口参数
    std::string serial_port; // 电机串口
    int baudrate = 115200;
    char parity = 'N';
    int data_bits = 8;
    int stop_bits = 1;
    // 网络参数
    uint16_t network_port; // 电机网络端口
    std::string motor_ip;  // 电机IP地址
    int slave_id = 1;      // 从站ID
    std::string byte_order = "big_endian"; // 字节序 ("big_endian" 或 "little_endian")

    // 雷达扫描参数
    size_t accumulated_frames = 2500; // 累积采集的帧数
    int angle_segments = 6;           // 角度分段数
    float angle_y_deg = 37.5f;        // Y轴旋转角度
    float voxel_size = 0.05f;         // 体素滤波大小

    // 保存点云参数
    bool is_save_pcd = false;                          // 是否保存PCD文件
    std::string save_path = "/opt/zwkj/pcds/lidar180"; // 点云保存路径

    friend std::ostream &operator<<(std::ostream &os, const LidarConfig &config)
    {
        os << "LidarConfig{sn=" << config.sn
           << ", type=" << static_cast<int>(config.type)
           << ", comm_type=" << config.comm_type
           << ", serial_port=" << config.serial_port
           << ", baudrate=" << config.baudrate
           << ", parity=" << config.parity
           << ", data_bits=" << config.data_bits
           << ", stop_bits=" << config.stop_bits
           << ", network_port=" << config.network_port
           << ", motor_ip=" << config.motor_ip
           << ", slave_id=" << config.slave_id
           << ", byte_order=" << config.byte_order
           << ", accumulated_frames=" << config.accumulated_frames
           << ", angle_segments=" << config.angle_segments
           << ", angle_y_deg=" << config.angle_y_deg
           << ", voxel_size=" << config.voxel_size
           << "}";
        return os;
    }
};