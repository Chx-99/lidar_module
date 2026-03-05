#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <memory>
#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <numbers>

#include "lidar_node/lidar_node.hpp"

/**
 * @brief LidarR1 雷达设备类（带电机控制的180度旋转雷达）
 * 
 * 功能：
 * - 分段旋转扫描（360度分为N段）
 * - 点云坐标变换与累积
 * - 体素滤波与直通滤波
 * - 点云发布
 */
class LidarR1 : public LidarNode
{
public:
    /**
     * @brief 构造函数
     * @param options ROS2节点选项
     * @param lidar_config 雷达配置
     * @param motor_config 电机配置
     * @param filter_config 滤波器配置
     * @throws std::runtime_error 如果必需的配置缺失
     */
    LidarR1(const rclcpp::NodeOptions &options, 
            const LidarConfig &lidar_config,
            const MotorConfig &motor_config, 
            const FilterConfig &filter_config)
        : LidarNode(options, lidar_config, motor_config, filter_config)
    {
        validateConfig();
        initializeMembers();
        createPublishers();
        logInitializationInfo();
    }

    ~LidarR1() override = default;

    /**
     * @brief 数据采集主函数（实现基类接口）
     * 
     * 执行完整的旋转扫描流程：
     * 1. 初始化电机并回归原点
     * 2. 在每个角度位置采集点云
     * 3. 应用滤波并发布累积点云
     * 
     * @throws std::runtime_error 如果电机或雷达操作失败
     */
    void collectData() override
    {
        if (!motor_) {
            throw std::runtime_error("电机未初始化，无法执行180度雷达采集");
        }

        const size_t segments = lidar_config_.angle_segments.value_or(6);
        RCLCPP_INFO(this->get_logger(), "[%s] 开始旋转扫描，分段数: %zu", 
                    lidar_config_.sn.c_str(), segments);

        // RAII 确保电机状态正确恢复
        accumulated_pointcloud_->clear();
        
        if (!initializeMotor()) {
            throw std::runtime_error("电机初始化失败");
        }

        // 执行分段采集
        for (size_t i = 0; i < segments; ++i)
        {
            const double target_angle = i * angle_step_;

            if (!collectPointCloudAtAngle(target_angle, i))
            {
                RCLCPP_ERROR(this->get_logger(), "[%s] 在角度 %.2f° 采集失败，终止扫描", 
                            lidar_config_.sn.c_str(), target_angle);
                throw std::runtime_error("点云采集失败");
            }
        }

        processAndPublishPointCloud();
        RCLCPP_INFO(this->get_logger(), "[%s] 旋转扫描完成", lidar_config_.sn.c_str());
    }

private:
    // ==================== 初始化相关 ====================
    
    /**
     * @brief 验证配置完整性
     * @throws std::runtime_error 如果必需的配置缺失
     */
    void validateConfig()
    {
        if (!lidar_config_.angle_segments.has_value()) {
            throw std::runtime_error("LidarR1 需要 angle_segments 配置");
        }
        if (!lidar_config_.motor_offset_y_angle.has_value()) {
            throw std::runtime_error("LidarR1 需要 motor_offset_y_angle 配置");
        }
        if (!lidar_config_.motor_bias_z_distance.has_value()) {
            throw std::runtime_error("LidarR1 需要 motor_bias_z_distance 配置");
        }
        if (*lidar_config_.angle_segments == 0) {
            throw std::runtime_error("angle_segments 不能为0");
        }
    }

    /**
     * @brief 初始化成员变量
     */
    void initializeMembers()
    {
        const size_t segments = *lidar_config_.angle_segments;
        angle_step_ = 360.0 / static_cast<double>(segments);

        accumulated_pointcloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        
        // 预留空间优化性能
        const size_t estimated_points = 96 * segments * lidar_config_.accumulated_frames;
        accumulated_pointcloud_->reserve(estimated_points);
    }

    /**
     * @brief 创建发布器
     */
    void createPublishers()
    {
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_" + lidar_config_.sn, 10);
    }

    /**
     * @brief 记录初始化信息
     */
    void logInitializationInfo() const
    {
        const size_t segments = *lidar_config_.angle_segments;
        const size_t total_points = 96 * segments * lidar_config_.accumulated_frames;
        
        RCLCPP_INFO(this->get_logger(), 
            "LidarR1 [%s] 初始化成功 - 帧数:%zu, 分段:%zu, 角度步长:%.2f°, 预计点数:%zu",
            lidar_config_.sn.c_str(), lidar_config_.accumulated_frames,
            segments, angle_step_, total_points);
    }

    /**
     * @brief 初始化电机状态
     * @return 成功返回true
     */
    bool initializeMotor()
    {
        try {
            motor_->returnToOrigin();
            motor_->enableMotor();
            RCLCPP_INFO(this->get_logger(), "[%s] 电机已回归原点并启动", 
                       lidar_config_.sn.c_str());
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[%s] 电机初始化失败: %s", 
                        lidar_config_.sn.c_str(), e.what());
            return false;
        }
    }

    // ==================== 采集相关 ====================
    /**
     * @brief 在指定角度采集点云
     * @param target_angle 目标角度（度）
     * @param segment_index 当前段索引
     * @return 成功返回true
     */
    [[nodiscard]] bool collectPointCloudAtAngle(double target_angle, size_t segment_index)
    {
        const size_t segments = *lidar_config_.angle_segments;
        RCLCPP_INFO(this->get_logger(), "[%s] [%zu/%zu] 移动到角度 %.2f°", 
                    lidar_config_.sn.c_str(), segment_index + 1, segments, target_angle);

        // 移动电机到目标角度
        double deviation = 0.0;
        if (!motor_->moveAbsoluteAndWait(target_angle, deviation, 
                                         motor_config_.motor_speed_rpm, motor_config_.motor_acceleration_rpm_s, POSITION_TOLERANCE))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 电机移动到 %.2f° 失败", 
                        lidar_config_.sn.c_str(), target_angle);
            return false;
        }

        RCLCPP_DEBUG(this->get_logger(), "[%s] 电机到达角度 %.2f°，偏差 %.2f°", 
                    lidar_config_.sn.c_str(), target_angle, deviation);

        // 采集并变换点云（补偿偏差）
        return captureAndTransformPointCloud(target_angle - deviation);
    }

    /**
     * @brief 采集并变换点云
     * @param current_angle 当前实际角度（已补偿偏差）
     * @return 成功返回true
     */
    [[nodiscard]] bool captureAndTransformPointCloud(double current_angle)
    {
        try
        {
            // 使用 lambda + unique_ptr 实现 RAII，确保激光状态正确
            if (!lidar_->enableLaser(LASER_RETRY_COUNT)) {
                throw std::runtime_error("激光开启失败");
            }
            
            // 采集点云数据
            auto cloud_msg = lidar_->getPointcloudData();
            if (!cloud_msg)
            {
                RCLCPP_ERROR(this->get_logger(), "[%s] 获取点云数据失败", 
                            lidar_config_.sn.c_str());
                return false;
            }

            lidar_->disableLaser(LASER_RETRY_COUNT); 

            // 坐标变换并累积
            transformAndAccumulatePointCloud(cloud_msg, current_angle);

            publishPointCloud(accumulated_pointcloud_); // 实时发布当前累积点云（可选）
            
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] 采集异常: %s", 
                        lidar_config_.sn.c_str(), e.what());
            return false;
        }
    }

    // ==================== 变换与滤波 ====================

    /**
     * @brief 对点云进行坐标变换并累积
     * @param cloud ROS点云消息
     * @param angle_deg 当前角度（度）
     */
    void transformAndAccumulatePointCloud(
        const sensor_msgs::msg::PointCloud2::SharedPtr &cloud, 
        double angle_deg)
    {
        // 转换为PCL格式
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*cloud, *pcl_cloud);

        RCLCPP_DEBUG(this->get_logger(), "[%s] 变换 %zu 个点，角度 %.2f°", 
                    lidar_config_.sn.c_str(), pcl_cloud->size(), angle_deg);

        // 获取变换矩阵（带缓存）
        const Eigen::Matrix4f& transform = getTransformMatrix(angle_deg);

        // 直接在目标点云中累积（避免中间副本）
        for (const auto &pt : pcl_cloud->points)
        {
            const Eigen::Vector4f point(pt.x, pt.y, pt.z, 1.0f);
            const Eigen::Vector4f transformed = transform * point;

            pcl::PointXYZI transformed_pt;
            transformed_pt.x = transformed(0);
            transformed_pt.y = transformed(1);
            transformed_pt.z = transformed(2);
            transformed_pt.intensity = pt.intensity;

            accumulated_pointcloud_->push_back(std::move(transformed_pt));
        }

        RCLCPP_DEBUG(this->get_logger(), "[%s] 累积点数: %zu",
                    lidar_config_.sn.c_str(), accumulated_pointcloud_->size());
    }

    /**
     * @brief 获取坐标变换矩阵（带缓存）
     * @param angle_deg 旋转角度（度）
     * @return 4x4变换矩阵的const引用
     */
    [[nodiscard]] const Eigen::Matrix4f& getTransformMatrix(double angle_deg) const
    {
        // 查找缓存
        if (auto it = transform_cache_.find(angle_deg); it != transform_cache_.end())
        {
            return it->second;
        }

        // 计算新矩阵
        const Eigen::Matrix4f transform = computeTransformMatrix(angle_deg);
        
        // 插入缓存并返回引用
        return transform_cache_.emplace(angle_deg, transform).first->second;
    }

    /**
     * @brief 计算坐标变换矩阵
     * @param angle_deg X轴旋转角度（度）
     * @return 4x4变换矩阵
     * 
     * 变换顺序：
     * 1. 绕Y轴旋转（电机偏移角）
     * 2. 沿Z轴平移（电机偏移距离）
     * 3. 沿X轴平移（电机偏移距离）
     * 4. 绕X轴旋转（当前扫描角度）
     */
    [[nodiscard]] Eigen::Matrix4f computeTransformMatrix(double angle_deg) const
    {
        using std::numbers::pi;
        
        // 1. Y轴旋转矩阵
        const double y_rad = lidar_config_.motor_offset_y_angle.value() * pi / 180.0;
        const float cos_y = static_cast<float>(std::cos(y_rad));
        const float sin_y = static_cast<float>(std::sin(y_rad));

        Eigen::Matrix4f R_y = Eigen::Matrix4f::Identity();
        R_y(0, 0) =  cos_y; R_y(0, 2) =  sin_y;
        R_y(2, 0) = -sin_y; R_y(2, 2) =  cos_y;

        // 2 Z轴平移矩阵 
        Eigen::Matrix4f T_z = Eigen::Matrix4f::Identity();
        T_z(2, 3) = static_cast<float>(lidar_config_.motor_bias_z_distance.value());
        // 3. X轴平移矩阵（电机偏移X距离）
        Eigen::Matrix4f T_x = Eigen::Matrix4f::Identity();
        T_x(0, 3) = static_cast<float>(lidar_config_.motor_bias_x_distance.value());

        

        // 4. X轴旋转矩阵
        const double x_rad = angle_deg * pi / 180.0;
        const float cos_x = static_cast<float>(std::cos(x_rad));
        const float sin_x = static_cast<float>(std::sin(x_rad));

        Eigen::Matrix4f R_x = Eigen::Matrix4f::Identity();
        R_x(1, 1) =  cos_x; R_x(1, 2) = -sin_x;
        R_x(2, 1) =  sin_x; R_x(2, 2) =  cos_x;

        // 组合变换：R_x * T_x * T_z * R_y
        return R_x * T_x * T_z * R_y;
    }

    // ==================== 发布相关 ====================

    /**
     * @brief 处理并发布累积点云
     * 
     * 流程：
     * 1. 应用直通滤波
     * 2. 应用体素滤波
     * 3. 应用世界坐标变换
     * 4. 发布ROS消息
     */
    void processAndPublishPointCloud()
    {
        if (accumulated_pointcloud_->empty())
        {
            RCLCPP_WARN(this->get_logger(), "[%s] 累积点云为空，无法发布", 
                       lidar_config_.sn.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[%s] 开始处理点云，原始点数: %zu",
                   lidar_config_.sn.c_str(), accumulated_pointcloud_->size());

        // // 测试点云，保存原始数据以便调试 /opt/zwkj/data/test/ sn + 时间戳.pcd (需要递归检查目录是否存在)
        // {
        //     auto filename = "/opt/zwkj/data/test/" + lidar_config_.sn + "_" + std::to_string(std::time(nullptr)) + ".pcd";
        //     // 递归检查目录是否存在，如果不存在则创建
        //     std::filesystem::path dir("/opt/zwkj/data/test/");
        //     if (!std::filesystem::exists(dir)) {
        //         std::filesystem::create_directories(dir);
        //     }

        //     pcl::io::savePCDFileBinary(filename, *accumulated_pointcloud_);
        //     RCLCPP_INFO(this->get_logger(), "[%s] 保存原始点云到文件: %s", 
        //                lidar_config_.sn.c_str(), filename.c_str());
        // }

        // 1. 应用直通滤波
        auto passthrough_filtered = applyPassThroughFilter(accumulated_pointcloud_);
        
            // 3. 应用世界坐标变换
        auto world_cloud = transformToWorld(passthrough_filtered);

        // 2. 应用体素滤波
        auto voxel_filtered = applyVoxelFilter(world_cloud);

        // 4. 发布·
        publishPointCloud(voxel_filtered);

        RCLCPP_INFO(this->get_logger(), "[%s] 点云发布完成，最终点数: %zu", 
                   lidar_config_.sn.c_str(), voxel_filtered->size());
    }

    /**
     * @brief 应用直通滤波器
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr 
    applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
    {
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        *filtered = *cloud; // 先复制

        for (const auto& filter_config : filter_config_.passthrough)
        {
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(filtered);
            pass.setFilterFieldName(filter_config.field_name);
            pass.setFilterLimits(filter_config.limits.first, filter_config.limits.second);
            
            auto temp = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pass.filter(*temp);
            filtered = temp;

            RCLCPP_DEBUG(this->get_logger(), "[%s] 直通滤波 [%s: %.2f~%.2f] 后点数: %zu",
                        lidar_config_.sn.c_str(), filter_config.field_name.c_str(),
                        filter_config.limits.first, filter_config.limits.second,
                        filtered->size());
        }

        return filtered;
    }

    /**
     * @brief 应用体素滤波
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr 
    applyVoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
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
                    lidar_config_.sn.c_str(), filter_config_.voxel_size, filtered->size());

        return filtered;
    }

    /**
     * @brief 应用世界坐标变换
     */
    [[nodiscard]] pcl::PointCloud<pcl::PointXYZI>::Ptr 
    transformToWorld(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
    {
        auto transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(*cloud, *transformed, filter_config_.transform_to_world);
        return transformed;
    }

    /**
     * @brief 发布点云到ROS话题
     */
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *msg);
        msg->header.frame_id = "map";
        msg->header.stamp = this->now();

        pointcloud_publisher_->publish(*msg);
    }
    // ==================== 常量定义 ====================

    static constexpr int LASER_RETRY_COUNT = 3;       // 激光操作重试次数
    static constexpr double POSITION_TOLERANCE = 0.5; // 位置到达容差（度）
    // ==================== 成员变量 ====================

    // 配置参数（从 lidar_config_ 提取）
    double angle_step_{0.0};           // 角度步长


    // 点云数据
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> accumulated_pointcloud_;

    // 变换矩阵缓存（使用 unordered_map 提高查找效率）
    mutable std::unordered_map<double, Eigen::Matrix4f> transform_cache_;

    // ROS发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};
