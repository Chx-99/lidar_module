#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/msg/string.hpp>

#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <map>

#include "lidar_node/lidar_node.hpp"
#include "config/config_struct.hpp"

using json = nlohmann::json;

/**
 * @brief LidarR1 雷达设备类（带电机控制的旋转雷达）
 *
 * 继承自 LidarBase（雷达功能）和 MotorBase（电机控制功能）
 * 实现旋转扫描功能
 */
class LidarR1 : public LidarNode
{
public:
    /**
     * @brief 构造函数
     * @param lidar_ip 雷达IP地址
     * @param local_ip 本地IP地址
     * @param sn 雷达序列号
     * @param comm Modbus通信接口
     * @param slave_id 电机从站ID
     * @param order 字节序
     * @param angle_segments 角度分段数（360度分成多少段）
     */
    LidarR1(const LidarConfig &config)
        : LidarNode(config),
          accumulated_frames_(config.accumulated_frames), angle_segments_(config.angle_segments), angle_step_(360.0 / config.angle_segments),
          voxel_size_(config.voxel_size), angle_y_deg_(config.angle_y_deg), is_save_pcd_(config.is_save_pcd), save_path_(config.save_path)
    {
        accumulated_pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_" + sn_, 10);
        RCLCPP_INFO(this->get_logger(), "LidarR1 [%s] 初始化成功，每帧点云数量: %zu，分段数: %d，角度步长: %.2f°", 
                   sn_.c_str(), 96 * angle_segments_ * accumulated_frames_, angle_segments_, angle_step_);
    }

    ~LidarR1() = default;

    /**
     * @brief 数据采集主函数（覆盖基类）
     *
     * 执行完整的旋转扫描流程：
     * 1. 初始化累积点云
     * 2. 在每个角度位置采集点云
     * 3. 发布最终的累积点云
     */
    void collect() override
    {
        RCLCPP_INFO(this->get_logger(), "[%s] 开始旋转扫描，分段数: %d", sn_.c_str(), angle_segments_);

        accumulated_pointcloud_->clear();
        motor_->returnToOrigin(); // 回归原点
        motor_->enableMotor();    // 启动电机

        for (int i = 0; i < angle_segments_; ++i)
        {
            double target_angle = i * angle_step_;

            if (!collectPointCloudAtAngle(target_angle, i))
            {
                RCLCPP_ERROR(this->get_logger(), "[%s] 在角度 %.2f° 采集失败，终止扫描", sn_.c_str(), target_angle);
                return;
            }
        }
        publishAccumulatedPointCloud();
        RCLCPP_INFO(this->get_logger(), "[%s] 旋转扫描完成", sn_.c_str());
    }

private:
    /**
     * @brief 在指定角度采集点云
     * @param target_angle 目标角度
     * @param segment_index 当前段索引（用于日志）
     * @return 成功返回true，失败返回false
     */
    bool collectPointCloudAtAngle(double target_angle, int segment_index)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] [%d/%d] 移动到角度 %.2f°", sn_.c_str(), segment_index + 1, angle_segments_, target_angle);

        double deviation = 0.0;
        // 1. 移动电机到目标角度
        if (!motor_->moveAbsoluteAndWait(target_angle, deviation, DEFAULT_MOTOR_SPEED, DEFAULT_MOTOR_ACCEL, POSITION_TOLERANCE))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 电机移动到 %.2f° 失败", sn_.c_str(), target_angle);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "[%s] [%d/%d] 电机已到达目标角度 %.2f°，偏差 %.2f°", sn_.c_str(), segment_index + 1, angle_segments_, target_angle, deviation);

        // 2. 采集并变换点云
        return captureAndTransformPointCloud(target_angle - deviation);
    }

    /**
     * @brief 采集并变换点云
     * @param current_angle 当前角度
     * @return 成功返回true，失败返回false
     */
    bool captureAndTransformPointCloud(double current_angle)
    {
        // 1. 开启激光
        if (!lidar_->enableLaser(LASER_TIMEOUT_SEC))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 激光开启失败", sn_.c_str());
            return false;
        }

        // 2. 采集点云数据
        auto cloud_msg = lidar_->getPointcloudData(accumulated_frames_);
        if (cloud_msg == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 获取点云数据失败", sn_.c_str());
            lidar_->disableLaser(LASER_TIMEOUT_SEC); // 尝试关闭激光
            return false;
        }
        // savePointCloudToPCD(cloud_msg, save_path_ + "-" + std::to_string(int(current_angle))); // 保存原始点云数据

        // 3. 关闭激光
        if (!lidar_->disableLaser(LASER_TIMEOUT_SEC))
        {
            RCLCPP_WARN(this->get_logger(), "[%s] 激光关闭失败", sn_.c_str());
            // 不返回false，因为数据已经采集成功
        }

        // 4. 坐标变换并累积
        transformAndAccumulatePointCloud(cloud_msg, current_angle);

        RCLCPP_DEBUG(this->get_logger(), "[%s] 角度 %.2f° 点云采集完成",
                     sn_.c_str(), current_angle);
        return true;
    }

    /**
     * @brief 对点云进行坐标变换并累积
     * @param cloud ROS点云消息
     * @param angle_deg 当前角度（度）
     */
    void transformAndAccumulatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud, double angle_deg)
    {
        // 转换为PCL格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud, *pcl_cloud);

        // 计算变换矩阵（带缓存）
        Eigen::Matrix4f transform = computeTransformMatrix(angle_deg);

        // 应用变换并直接累积到accumulated_pointcloud_
        // 避免创建中间的transformed_cloud，直接在循环中累积
        for (const auto &pt : pcl_cloud->points)
        {
            Eigen::Vector4f point(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f transformed_point = transform * point;

            pcl::PointXYZI transformed_pt;
            transformed_pt.x = transformed_point(0);
            transformed_pt.y = transformed_point(1);
            transformed_pt.z = transformed_point(2);
            transformed_pt.intensity = pt.intensity;

            accumulated_pointcloud_->push_back(transformed_pt);
        }

        RCLCPP_DEBUG(this->get_logger(), "[%s] 点云变换完成，当前累积点数: %zu",
                     sn_.c_str(), accumulated_pointcloud_->size());
    }

    /**
     * @brief 计算坐标变换矩阵（带缓存优化）
     * @param angle_deg 旋转角度（度）
     * @return 4x4变换矩阵
     */
    Eigen::Matrix4f computeTransformMatrix(double angle_deg) const
    {
        // 检查缓存
        auto it = transform_cache_.find(angle_deg);
        if (it != transform_cache_.end())
        {
            return it->second;
        }

        // 1. 绕Y轴旋转temp_angle_deg度
        double y_angle_rad = angle_y_deg_ * M_PI / 180.0;
        float cos_y = std::cos(y_angle_rad);
        float sin_y = std::sin(y_angle_rad);

        Eigen::Matrix4f R_y = Eigen::Matrix4f::Identity();
        R_y << cos_y, 0.0f, sin_y, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            -sin_y, 0.0f, cos_y, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;

        // 2. 沿Z轴平移10mm (0.01m)
        Eigen::Matrix4f T_z = Eigen::Matrix4f::Identity();
        T_z(2, 3) = -0.01f; // Z轴平移10mm

        // 3. 绕X轴旋转-angle_deg度
        double x_angle_rad = angle_deg * M_PI / 180.0;
        float cos_x = std::cos(x_angle_rad);
        float sin_x = std::sin(x_angle_rad);

        Eigen::Matrix4f R_x = Eigen::Matrix4f::Identity();
        R_x << 1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, cos_x, -sin_x, 0.0f,
            0.0f, sin_x, cos_x, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f;

        // 组合变换：先Y轴旋转，再Z轴平移，最后X轴旋转
        Eigen::Matrix4f result = R_x * T_z * R_y;

        // 存入缓存
        transform_cache_[angle_deg] = result;

        return result;
    }

    /**
     * @brief 发布累积的点云
     */
    void publishAccumulatedPointCloud()
    {
        if (accumulated_pointcloud_->empty())
        {
            RCLCPP_WARN(this->get_logger(), "[%s] 累积点云为空，无法发布", sn_.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(accumulated_pointcloud_);
        float voxel_size = 0.05f; // 5cm体素大小
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.setDownsampleAllData(true);
        voxel_filter.filter(*filtered_cloud);

        // 保存PCD文件（如果需要）
        if (is_save_pcd_)
        {
            savePointCloudToPCD(filtered_cloud, save_path_);
            RCLCPP_INFO(this->get_logger(), "[%s] 累积点云已保存到文件：%s", sn_.c_str(), save_path_.c_str());
        }

        // 转换为ROS消息
        auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*filtered_cloud, *output_msg);
        output_msg->header.frame_id = "map";
        output_msg->header.stamp = this->now();

        
        
        // 发布点云
        pointcloud_publisher_->publish(*output_msg);
        RCLCPP_INFO(this->get_logger(), "[%s] 点云发布完成，总点数: %zu", sn_.c_str(), filtered_cloud->size());
    }

private:
    static constexpr int LASER_TIMEOUT_SEC = 10;       // 激光操作超时时间（秒）
    static constexpr double POSITION_TOLERANCE = 0.5;  // 位置到达容差（度）
    static constexpr double DEFAULT_MOTOR_SPEED = 5.0; // 默认电机速度（RPM）
    static constexpr double DEFAULT_MOTOR_ACCEL = 1.0; // 默认电机加速度（RPM/s）

    const size_t accumulated_frames_; // 累积采集的帧数
    const int angle_segments_;        // 角度分段数
    const double angle_step_;         // 每段角度步长
    const float voxel_size_;          // 体素滤波大小
    const float angle_y_deg_;         // 绕Y轴旋转角度
    const bool is_save_pcd_;          // 是否保存点云文件
    const std::string save_path_;     // 点云保存路径

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> accumulated_pointcloud_; // 累积的点云

    // 缓存的变换矩阵，避免重复计算三角函数
    mutable std::map<double, Eigen::Matrix4f> transform_cache_; // 变换矩阵缓存

    // 发布每帧点云的Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};
