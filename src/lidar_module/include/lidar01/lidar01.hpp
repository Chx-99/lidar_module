#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "sensor/lidar/lidar_base.hpp"
#include "sensor/lidar/base/data_struct.hpp"
#include "blockingconcurrentqueue.h"

/**
 * @brief Lidar01 雷达设备类
 */
class Lidar01 : public lidar_base::LidarBase
{
public:
    Lidar01(const std::string &lidar_ip, const std::string &local_ip, const std::string &sn)
        : lidar_base::LidarBase(lidar_ip, local_ip, sn) {}
    ~Lidar01() = default;

    void collect() override
    {
        std::string sn = getSN();

        // 1、开启雷达与IMU数据采集
        if (enableLaser() == false)
        {
            RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ] 激光开启失败，无法进行数据采集", sn.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "雷达设备[ %s ] 激光已开启，开始采集数据", sn.c_str());

        // 2、采集点云数据
        std::vector<std::pair<double, base_frame::DataFrame<base_frame::SingleEchoRectangularData, 96>>> batch;
        batch.resize(accumulate_frames_); // 必须resize分配空间
        auto &pointcloud_queue = getPointcloudQueue();
        // 3、等待直到收集到足够的帧数
        size_t total_received = 0;
        while (total_received < accumulate_frames_)
        {
            auto length = pointcloud_queue.wait_dequeue_bulk(batch.begin() + total_received, accumulate_frames_ - total_received);
            total_received += length;
        }

        // 4、处理点云数据
        auto msg = frame_tools::convertToPointCloud2(batch, sn);
        pointcloud_pub_->publish(*msg);

        // 5、停止采集
        if (disableLaser() == false)
        {
            RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ] 激光关闭失败", sn.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "雷达设备[ %s ] 激光已关闭，停止数据采集", sn.c_str());
    }

private:
    static constexpr size_t accumulate_frames_ = 2500; // 累积帧数阈值
};
