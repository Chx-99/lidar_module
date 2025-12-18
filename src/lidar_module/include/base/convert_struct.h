/**
 * @brief imu与点云转换
 *
 * @file convert_struct.h
 * @author cao haoxuan
 * @date 2025-12-16
 */

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

#include "data_struct.h"


// TODO: 未完成

// ===================================chx修改部分===============================================
inline sensor_msgs::msg::PointCloud2 convertToPointCloud2(
    const rclcpp::Clock::SharedPtr &clock,
    std::shared_ptr<std::vector<dataFrame<singleEchoRectangularData, 96>>> data,
    const std::string &sn = "") {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = clock->now();
  // cloud_msg.header.stamp = rclcpp::Time((*data)[0].timestamp);
  cloud_msg.header.frame_id = "lidar" + (sn.empty() ? "" : "_" + sn);

  // 定义字段
  cloud_msg.fields.resize(7);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[0].count = 1;

  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[1].count = 1;

  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[2].count = 1;

  cloud_msg.fields[3].name = "intensity";
  cloud_msg.fields[3].offset = 12;
  cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud_msg.fields[3].count = 1;

  cloud_msg.fields[4].name = "timestamp";
  cloud_msg.fields[4].offset = 16;
  cloud_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud_msg.fields[4].count = 1;

  cloud_msg.fields[5].name = "tag";
  cloud_msg.fields[5].offset = 20;
  cloud_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud_msg.fields[5].count = 1;

  cloud_msg.fields[6].name = "line";
  cloud_msg.fields[6].offset = 21;
  cloud_msg.fields[6].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud_msg.fields[6].count = 1;

  // 预分配最大可能的内存
  size_t max_points = data->size() * 96;
  cloud_msg.height = 1;
  cloud_msg.width = max_points;
  cloud_msg.is_dense = true;
  cloud_msg.point_step = 22;
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.width * cloud_msg.point_step);

  // 创建迭代器
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg,
                                                         "intensity");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_timestamp(cloud_msg,
                                                            "timestamp");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(cloud_msg, "tag");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_line(cloud_msg, "line");

  size_t valid_count = 0;
  uint32_t current_point_timestamp_us = 0;
  // 单循环处理并直接写入
  for (const auto &packet : *data) {
    for (const auto &point : packet.data) {
      float x = point.x / 1000.f;
      float y = point.y / 1000.f;
      float z = point.z / 1000.f;

      *iter_x = x;
      *iter_y = y;
      *iter_z = z;
      *iter_intensity = static_cast<float>(point.intensity);
      // 转换为微秒
      *iter_timestamp = current_point_timestamp_us;
      *iter_tag = point.lable & 0x0F;
      *iter_line = (point.lable >> 4) & 0x0F;

      // std::cout  << "x: " << *iter_x << "  y: " << *iter_y << "  z: " <<
      // *iter_z
      // << "  intensity: " << std::hex << *iter_intensity << "  timestamp: " <<
      // std::hex << *iter_timestamp
      // << "  tag: " << std::hex << static_cast<int>(*iter_tag)
      // << "  line: " << std::hex << std::hex << static_cast<int>(*iter_line)
      // << std::endl;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
      ++iter_timestamp;
      ++iter_tag;
      ++iter_line;
      ++valid_count;

      current_point_timestamp_us += 10;
    }
  }

  // 调整到实际大小
  cloud_msg.width = valid_count;
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step);

  return cloud_msg;
}

inline sensor_msgs::msg::Imu convertToIMU(
    const rclcpp::Clock::SharedPtr &clock,
    std::shared_ptr<dataFrame<imuData, 1>> data, const std::string &sn = "") {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = clock->now();
  // imu_msg.header.stamp = rclcpp::Time(data->timestamp);
  imu_msg.header.frame_id = "lidar";

  imuData &imu_data = data->data[0];

  imu_msg.angular_velocity.x = imu_data.gyro_x;
  imu_msg.angular_velocity.y = imu_data.gyro_y;
  imu_msg.angular_velocity.z = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x;
  imu_msg.linear_acceleration.y = imu_data.acc_y;
  imu_msg.linear_acceleration.z = imu_data.acc_z;

  return imu_msg;
}

// ===========================================================================================