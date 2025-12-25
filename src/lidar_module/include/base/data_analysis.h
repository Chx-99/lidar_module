#include "base/data_struct.h"
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <functional>
using namespace base_frame;
using namespace frame_tools;

// 回调函数类型
using AckCallback = std::function<void(const std::string &sn, const std::vector<uint8_t> &data)>;
using PointCloudBatchCallback = std::function<void(const std::string &sn, const std::vector<std::shared_ptr<std::vector<uint8_t>>> &batch)>;
using IMUCallback = std::function<void(const std::string &sn, const std::vector<uint8_t> &data)>;

void ack_process(const std::string &sn, const std::vector<uint8_t> &data)
{
    auto ack_tab = getAckNameCompileTime(GET_ACK_SET(data.data()), GET_ACK_ID(data.data()));

    // 使用if-else替代switch，因为ack_tab是字符串
    if (ack_tab == "HandShake ACK")
    {
        HandShakeACK handshake = fromVector<HandShakeACK>(data);
        std::cout << "[" << sn << "] 收到 HandShake ACK: " << handshake << std::endl;
    }
    else if (ack_tab == "HeartBeat ACK")
    {
        // HeartBeatACK heartbeat = fromVector<HeartBeatACK>(packet.data);
        // std::cout << "[" << sn << "] 收到 HeartBeat ACK: " << heartbeat << std::endl;
    }
    else if (ack_tab == "SetLaserStatus ACK")
    {
        SetLaserStatusACK set_laser_status = fromVector<SetLaserStatusACK>(data);
        std::cout << "[" << sn << "] 收到 SetLaserStatus ACK: " << set_laser_status << std::endl;
    }
    else if (ack_tab == "Set IMU Frequency ACK")
    {
        SetIMUFrequencyACK set_imu_frequency = fromVector<SetIMUFrequencyACK>(data);
        std::cout << "[" << sn << "] 收到 Set IMU Frequency ACK: " << set_imu_frequency << std::endl;
    }
    else
    {
        std::cout << "[" << sn << "] 收到未知 ACK/MSG" << std::endl;
    }
}

void imu_process(const std::string &sn, const std::vector<uint8_t> &data)
{
    // 处理IMU数据（使用shared_ptr，零拷贝）
    // std::cout << "[" << sn << "] 收到IMU数据包，大小: " << data.size() << " 字节" << std::endl;
    // 在这里添加你的IMU处理算法
    auto imu_frame = fromVector<dataFrame<ImuData>>(data);
    // std::cout << "[" << sn << "] 处理完成IMU数据，时间戳: " << imu_frame.timestamp << std::endl;
}

void pointcloud_process(const std::string &sn, const std::vector<std::shared_ptr<std::vector<uint8_t>>> &batch)
{
    // 使用局部变量而非thread_local，每次自动释放
    std::vector<SingleEchoRectangularData> pointcloud_points;
    pointcloud_points.reserve(batch.size() * 100); // 预估容量，仅本次使用

    for (const auto &pc_ptr : batch)
    {
        auto frame = fromVector<dataFrame<SingleEchoRectangularData>>(*pc_ptr);
        pointcloud_points.insert(pointcloud_points.end(), frame.datas.begin(), frame.datas.end());
    }

    // 在这里添加你的点云处理算法
    // std::cout << "[" << sn << "] 处理点云: " << pointcloud_points.size() << " 点" << std::endl;

    // 函数结束时自动释放内存
}