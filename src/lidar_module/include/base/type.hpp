#pragma once

#include <string>
#include <optional>
#include <vector>
#include <Eigen/Dense>

#include "sensor/motor/base/plc_device_base.hpp"

/**
 * @brief 雷达类型（使用强类型枚举）
 */
enum class LidarType : uint8_t
{
    LIDAR_R1 = 0, // 180度雷达（需要电机）
    LIDAR_80 = 1   // 80度雷达（无需电机）
};

/**
 * @brief 直通滤波器配置
 */
struct PassThroughFilter
{
    std::string field_name;         // 字段名 (x/y/z)
    std::pair<float, float> limits; // 过滤范围
};

/**
 * @brief 电机配置（仅180度雷达需要）
 */
struct MotorConfig
{
    std::string comm_type;                       // 通信类型: "rtu", "tcp", "ttw"
    ByteOrder byte_order = ByteOrder::BigEndian; // 字节序
    uint8_t slave_id = 1;                        // Modbus从站ID

    float motor_speed_rpm = 5.0f;              // 电机转速（RPM）
    float motor_acceleration_rpm_s = 1.0f;     // 电机加速度（RPM/s）

    // RTU 串口配置（comm_type == "rtu" 时需要）
    std::optional<std::string> serial_port; // 串口名称
    std::optional<uint32_t> baudrate;       // 波特率（默认9600）
    std::optional<char> parity;             // 奇偶校验: 'N', 'E', 'O'
    std::optional<uint8_t> data_bits;       // 数据位（默认8）
    std::optional<uint8_t> stop_bits;       // 停止位（默认1）

    // TCP/TTW 网络配置（comm_type == "tcp" 或 "ttw" 时需要）
    std::optional<std::string> motor_ip;  // 电机IP地址
    std::optional<uint16_t> network_port; // 网络端口
};

/**
 * @brief 雷达配置
 */
struct LidarConfig
{
    std::string sn;       // 雷达序列号
    std::string lidar_ip; // 雷达IP地址
    std::string local_ip; // 本地网卡IP地址
    LidarType type;       // 雷达类型

    // 雷达采集参数
    size_t accumulated_frames = 2500; // 累积帧数
    bool repetitive_scan = false; // 是否启用重复扫描机制

    // 180度雷达特有参数
    std::optional<size_t> angle_segments;       // 角度分段数
    std::optional<float> motor_offset_y_angle;  // 电机Y轴偏移角度
    std::optional<float> motor_bias_z_distance; // 电机Z轴偏移距离
    std::optional<float> motor_bias_x_distance; // 电机X轴偏移距离
};

/**
 * @brief 滤波器配置
 */
struct FilterConfig
{
    Eigen::Matrix4f transform_to_world = Eigen::Matrix4f::Identity(); // 雷达到世界坐标系的变换矩阵
    float voxel_size = 0.05f;                                         // 体素滤波大小（米）
    std::vector<PassThroughFilter> passthrough;                       // 直通滤波器列表
};
