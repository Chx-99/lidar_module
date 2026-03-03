#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <stdexcept>

#include "lidar_node/lidar_node.hpp"

/**
 * @brief Lidar01 雷达设备类（80度固定视场角雷达）
 * 
 * 功能：
 * - 单帧点云采集
 * - 直通滤波与体素滤波
 * - 世界坐标系变换
 * - 点云发布
 */
class Lidar01 : public LidarNode
{
public:
    /**
     * @brief 构造函数
     * @param options ROS2节点选项
     * @param lidar_config 雷达配置
     * @param motor_config 电机配置（80度雷达不使用，传入空配置）
     * @param filter_config 滤波器配置
     * @throws std::runtime_error 如果配置验证失败
     */
    Lidar01(const rclcpp::NodeOptions &options,
            const LidarConfig &lidar_config,
            const MotorConfig &motor_config,
            const FilterConfig &filter_config)
        : LidarNode(options, lidar_config, motor_config, filter_config)
    {
        validateConfig();
        createPublisher();
        logInitializationInfo();
    }

    ~Lidar01() override = default;

    /**
     * @brief 数据采集主函数（实现基类接口）
     * 
     * 执行流程：
     * 1. 启用激光并采集点云
     * 2. 应用滤波器处理
     * 3. 发布处理后的点云
     * 
     * @throws std::runtime_error 如果雷达操作失败
     */
    void collectData() override
    {
        if (!lidar_)
        {
            throw std::runtime_error("雷达未初始化，无法执行数据采集");
        }

        RCLCPP_INFO(this->get_logger(), "[%s] 开始采集点云数据", 
                    lidar_config_.sn.c_str());

        // 采集原始点云（带 RAII 激光控制）
        auto raw_cloud_msg = captureRawPointCloud();
        if (!raw_cloud_msg)
        {
            throw std::runtime_error("点云采集失败");
        }

        // 处理并发布点云
        processAndPublishPointCloud(raw_cloud_msg);

        RCLCPP_INFO(this->get_logger(), "[%s] 点云采集完成", 
                    lidar_config_.sn.c_str());
    }

private:
    // ==================== 初始化相关 ====================

    /**
     * @brief 验证配置完整性
     * @throws std::runtime_error 如果配置不符合要求
     */
    void validateConfig()
    {
        if (lidar_config_.type != LidarType::LIDAR_80)
        {
            throw std::runtime_error("Lidar01 只支持 LIDAR_80 类型");
        }

        if (lidar_config_.accumulated_frames == 0)
        {
            throw std::runtime_error("accumulated_frames 不能为0");
        }
    }

    /**
     * @brief 创建ROS发布器
     */
    void createPublisher()
    {
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_" + lidar_config_.sn, 10);
    }

    /**
     * @brief 记录初始化信息
     */
    void logInitializationInfo() const
    {
        const size_t estimated_points = 96 * lidar_config_.accumulated_frames;
        RCLCPP_INFO(this->get_logger(),
                    "Lidar01 [%s] 初始化成功 - 帧数:%zu, 预计点数:%zu",
                    lidar_config_.sn.c_str(), 
                    lidar_config_.accumulated_frames,
                    estimated_points);
    }

    // ==================== 采集相关 ====================

    /**
     * @brief 采集原始点云数据
     * @return ROS点云消息（失败返回nullptr）
     */
    [[nodiscard]] sensor_msgs::msg::PointCloud2::SharedPtr captureRawPointCloud()
    {
        // 启用激光
        if (!lidar_->enableLaser(LASER_RETRY_COUNT))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 激光开启失败", 
                        lidar_config_.sn.c_str());
            return nullptr;
        }

        RCLCPP_DEBUG(this->get_logger(), "[%s] 激光已开启，开始采集", 
                    lidar_config_.sn.c_str());

        // 采集点云数据
        auto cloud_msg = lidar_->getPointcloudData();
        if (!cloud_msg)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 获取点云数据失败", 
                        lidar_config_.sn.c_str());
            return nullptr;
        }

        // 使用 lambda + unique_ptr 实现 RAII，确保激光关闭
        auto laser_cleanup = [this](int*) {
            if (!lidar_->disableLaser(LASER_RETRY_COUNT))
            {
                RCLCPP_ERROR(this->get_logger(), "[%s] 激光关闭失败", 
                            lidar_config_.sn.c_str());
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "[%s] 激光已关闭", 
                            lidar_config_.sn.c_str());
            }
        };
        std::unique_ptr<int, decltype(laser_cleanup)> laser_guard(nullptr, laser_cleanup);

        RCLCPP_DEBUG(this->get_logger(), "[%s] 采集到 %zu 个点", 
                    lidar_config_.sn.c_str(), 
                    cloud_msg->data.size() / cloud_msg->point_step);

        return cloud_msg;
    }

    // ==================== 处理与发布 ====================

    /**
     * @brief 处理并发布点云
     * @param cloud_msg 原始ROS点云消息
     */
    void processAndPublishPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg)
    {
        // 转换为PCL格式
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        RCLCPP_DEBUG(this->get_logger(), "[%s] 开始处理点云，原始点数: %zu",
                    lidar_config_.sn.c_str(), pcl_cloud->size());

        // 1. 应用直通滤波
        auto passthrough_filtered = applyPassThroughFilter(pcl_cloud);

        // 2. 应用体素滤波
        auto voxel_filtered = applyVoxelFilter(passthrough_filtered);

        // 3. 应用世界坐标变换
        auto world_cloud = transformToWorld(voxel_filtered);

        // 4. 发布
        publishPointCloud(world_cloud);

        RCLCPP_INFO(this->get_logger(), "[%s] 点云发布完成，最终点数: %zu",
                   lidar_config_.sn.c_str(), world_cloud->size());
    }

    /**
     * @brief 应用直通滤波器
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr
    applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
    {
        if (filter_config_.passthrough.empty())
        {
            return cloud; // 无滤波器时直接返回
        }

        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        *filtered = *cloud;

        for (const auto &filter_cfg : filter_config_.passthrough)
        {
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(filtered);
            pass.setFilterFieldName(filter_cfg.field_name);
            pass.setFilterLimits(filter_cfg.limits.first, filter_cfg.limits.second);

            auto temp = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pass.filter(*temp);
            filtered = temp;

            RCLCPP_DEBUG(this->get_logger(), "[%s] 直通滤波 [%s: %.2f~%.2f] 后点数: %zu",
                        lidar_config_.sn.c_str(), filter_cfg.field_name.c_str(),
                        filter_cfg.limits.first, filter_cfg.limits.second,
                        filtered->size());
        }

        return filtered;
    }

    /**
     * @brief 应用体素滤波
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr
    applyVoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
    {
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(filter_config_.voxel_size,
                                  filter_config_.voxel_size,
                                  filter_config_.voxel_size);
        voxel_filter.setDownsampleAllData(true);
        voxel_filter.filter(*filtered);

        RCLCPP_DEBUG(this->get_logger(), "[%s] 体素滤波 [%.3fm] 后点数: %zu",
                    lidar_config_.sn.c_str(), filter_config_.voxel_size, 
                    filtered->size());

        return filtered;
    }

    /**
     * @brief 应用世界坐标变换
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr
    transformToWorld(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
    {
        auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(*cloud, *transformed, filter_config_.transform_to_world);
        return transformed;
    }

    /**
     * @brief 发布点云到ROS话题
     */
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        //测试保存pcd文件 /home/zwkj01/sn_+时间戳.pcd
        // std::string filename = "/home/zwkj01/sn_" + lidar_config_.sn + "_" + std::to_string(this->now().nanoseconds()) + ".pcd";
        // pcl::io::savePCDFileBinary(filename, *cloud);
        // RCLCPP_INFO(this->get_logger(), "[%s] 点云已保存到: %s", lidar_config_.sn.c_str(), filename.c_str());

        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *msg);
        msg->header.frame_id = "map";
        msg->header.stamp = this->now();

        pointcloud_publisher_->publish(*msg);
    }

    // ==================== 常量定义 ====================
    static constexpr int LASER_RETRY_COUNT = 10; // 激光操作重试次数

    // ==================== 成员变量 ====================
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};
