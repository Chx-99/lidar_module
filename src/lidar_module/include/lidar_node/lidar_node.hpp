#pragma once

#include <boost/asio.hpp>

#include "sensor/motor/base/comm_interface.hpp"
#include "sensor/motor/base/plc_device_base.hpp"
#include "sensor/motor/base/modbus_rtu.hpp"
#include "sensor/motor/base/modbus_tcp.hpp"
#include "sensor/motor/base/transparent_transport_wrapper.hpp"
#include "sensor/lidar/lidar_base.hpp"
#include "sensor/motor/motor_base.hpp"
#include "config/config_struct.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

class LidarNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    LidarNode(const LidarConfig &config)
        : rclcpp::Node("lidar_node_" + config.sn), sn_(config.sn), lidar_type_(config.type),
          is_start_task_(false), work_context_(), work_guard_(boost::asio::make_work_guard(work_context_))
    {
        lidar_ = std::make_unique<lidar_base::LidarBase>(config.lidar_ip, config.local_ip, config.sn);
        if (config.type == LidarType::LIDAR_180)
        {
            // 根据配置创建电机对象
            ByteOrder byte_order = config.byte_order == "big_endian" ? ByteOrder::BigEndian : ByteOrder::LittleEndian;
            if (config.comm_type == "rtu")
            {
                auto modbus_rtu = std::make_shared<ModbusRTU>(config.serial_port, config.baudrate, config.parity, config.data_bits, config.stop_bits);
                motor_ = std::make_unique<motor_base::MotorBase>(modbus_rtu, config.slave_id, byte_order);
            }
            else if (config.comm_type == "tcp")
            {
                auto modbus_tcp = std::make_shared<ModbusTCP>(config.motor_ip, config.network_port);
                motor_ = std::make_unique<motor_base::MotorBase>(modbus_tcp, config.slave_id, byte_order);
            }
            else if (config.comm_type == "ttw")
            {
                auto ttw = std::make_shared<TransparentTransportWrapper>(config.motor_ip, config.network_port);
                motor_ = std::make_unique<motor_base::MotorBase>(ttw, config.slave_id, byte_order);
            }
            else
            {
                throw std::runtime_error("未知的电机通信类型: " + config.comm_type);
            }
        }
        else
        {
            motor_ = nullptr; // 80度雷达不需要电机
        }
        // 初始化采集服务
        collect_service_ = this->create_service<std_srvs::srv::Trigger>("collect_" + sn_, std::bind(&LidarNode::handleTrigger, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Lidar[%s] 已初始化，等待采集请求...", sn_.c_str());
        RCLCPP_DEBUG(this->get_logger(), "Lidar[%s] 采集服务名称：collect_%s", sn_.c_str(), sn_.c_str());
        worker_thread_ = std::thread([this]()
                                     { work_context_.run(); });
    }

    ~LidarNode()
    {
        // 停止工作上下文
        work_guard_.reset();
        work_context_.stop();
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Lidar[%s] 节点已销毁", lidar_->getSN().c_str());
    }

protected:
    /**
     * @brief 采集数据接口（由子类实现具体逻辑）
     */
    virtual void collect() = 0;

    void savePointCloudToPCD(const sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud, const std::string &save_path)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointcloud, *cloud);
        // 确保保存路径存在
        std::string full_save_path = save_path;
        if (full_save_path.back() != '/')
        {
            full_save_path += '/';
        }
        if (!std::filesystem::exists(full_save_path))
        {
            std::filesystem::create_directories(full_save_path);
        }

        // 生成带时间戳的文件名
        std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm now_tm = *std::localtime(&now_c);
        char date_str[20];
        std::strftime(date_str, sizeof(date_str), "%Y%m%d-%H%M%S", &now_tm);
        std::string pcd_filename = full_save_path + "lidar_" + sn_ + "_" + std::string(date_str) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_filename, *cloud);
        RCLCPP_INFO(this->get_logger(), "[%s] 累积点云已保存到 %s", sn_.c_str(), pcd_filename.c_str());
    }

    void savePointCloudToPCD(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::string &save_path)
    {
        // 确保保存路径存在
        std::string full_save_path = save_path;
        if (full_save_path.back() != '/')
        {
            full_save_path += '/';
        }
        if (!std::filesystem::exists(full_save_path))
        {
            std::filesystem::create_directories(full_save_path);
        }

        // 生成带时间戳的文件名
        std::time_t now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm now_tm = *std::localtime(&now_c);
        char date_str[20];
        std::strftime(date_str, sizeof(date_str), "%Y%m%d-%H%M%S", &now_tm);
        std::string pcd_filename = full_save_path + "lidar_" + sn_ + "_" + std::string(date_str) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_filename, *cloud);
        RCLCPP_INFO(this->get_logger(), "[%s] 累积点云已保存到 %s", sn_.c_str(), pcd_filename.c_str());
    }

private:
    /**
     * @brief 处理采集请求服务回调
     */
    void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            if (is_start_task_.load(std::memory_order_acquire))
            {
                RCLCPP_WARN(this->get_logger(), "当前正在采集数据，拒绝新的采集请求");
                response->success = false;
                response->message = "当前正在采集数据，拒绝新的采集请求";
                return;
            }
            else
            {
                is_start_task_.store(true, std::memory_order_release);
                RCLCPP_INFO(this->get_logger(), "当前未采集数据，准备开始采集");
            }
        }
        boost::asio::post(work_context_, [this]()
                          {
                    RCLCPP_INFO(this->get_logger(), "Lidar[%s] 采集任务已启动", lidar_->getSN().c_str());
                    try
                    {
                        collect();
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Lidar[%s] 采集任务异常终止：%s", lidar_->getSN().c_str(), e.what());
                    }
                    is_start_task_.store(false, std::memory_order_release);
                    RCLCPP_INFO(this->get_logger(), "Lidar[%s] 采集任务已结束", lidar_->getSN().c_str()); });
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 确保任务已提交
        response->success = true;
        response->message = "采集命令已接受，正在启动采集";
        return;
    }

protected:
    const std::string sn_;
    const LidarType lidar_type_;
    std::unique_ptr<lidar_base::LidarBase> lidar_;
    std::unique_ptr<motor_base::MotorBase> motor_;

private:
    std::mutex task_mutex_;
    std::atomic<bool> is_start_task_; // 采集任务标志位
    boost::asio::io_context work_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread worker_thread_;
    // 采集服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr collect_service_;
};