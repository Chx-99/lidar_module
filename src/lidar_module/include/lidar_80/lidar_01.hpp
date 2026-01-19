#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <chrono>
#include <filesystem>

#include "lidar_node/lidar_node.hpp"
#include "config/config_struct.hpp"

/**
 * @brief Lidar01 雷达设备类
 */
class Lidar01 : public LidarNode
{
public:
    Lidar01(const LidarConfig &config)
        : LidarNode(config), accumulate_frames_(config.accumulated_frames),
          is_save_pcd_(config.is_save_pcd), save_path_(config.save_path)
    {
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_" + sn_, 10);
        RCLCPP_INFO(this->get_logger(), "Lidar01 [%s] 初始化成功，每帧点云数量: %zu", sn_.c_str(), 96 * accumulate_frames_);
    }
    ~Lidar01() = default;

    void collect() override
    {
        // 1、开启雷达与IMU数据采集
        if (lidar_->enableLaser(10) == false)
        {
            RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ] 激光开启失败，无法进行数据采集", sn_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "雷达设备[ %s ] 激光已开启，开始采集数据", sn_.c_str());
        // 2、采集点云数据并发布
        auto msg = lidar_->getPointcloudData(accumulate_frames_);
        if (msg == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ] 获取点云数据失败", sn_.c_str());
            return;
        }
        pointcloud_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "雷达设备[ %s ] 发布点云数据，点云数量：%zu", sn_.c_str(), msg->data.size() / msg->point_step);
        // 3、停止采集
        if (lidar_->disableLaser(10) == false)
        {
            RCLCPP_ERROR(this->get_logger(), "雷达设备[ %s ] 激光关闭失败", sn_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "雷达设备[ %s ] 激光已关闭，停止数据采集", sn_.c_str());
        // 4、保存PCD文件（如果配置了保存）
        if (is_save_pcd_)
        {
            savePointCloudToPCD(msg, save_path_);
            RCLCPP_INFO(this->get_logger(), "雷达设备[ %s ] 点云数据已保存到文件：%s", sn_.c_str(), save_path_.c_str());
        }
    }

private:
    const size_t accumulate_frames_; // 累积帧数阈值
    const bool is_save_pcd_;         // 是否保存PCD文件
    const std::string save_path_;    // 点云保存路径

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};
