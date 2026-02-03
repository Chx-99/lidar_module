#pragma once

#include <boost/asio.hpp>

#include "sensor/motor/base/comm_interface.hpp"
#include "sensor/motor/base/plc_device_base.hpp"
#include "sensor/motor/base/modbus_rtu.hpp"
#include "sensor/motor/base/modbus_tcp.hpp"
#include "sensor/motor/base/transparent_transport_wrapper.hpp"
#include "sensor/lidar/lidar_base.hpp"
#include "sensor/motor/motor_base.hpp"
#include "base/type.hpp"
#include "base/alternate_scheduler.hpp"

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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
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
    LidarNode(const rclcpp::NodeOptions &options, const LidarConfig &lidar_config,
              const MotorConfig &motor_config, const FilterConfig &filter_config)
        : rclcpp::Node("Lidar_" + lidar_config.sn, options), lidar_config_(lidar_config),
          motor_config_(motor_config), filter_config_(filter_config), is_start_task_(false)
    {
        initAlt();
        initDevices();
        
        // 启动交替采集调度器
        if (alt_scheduler_)
        {
            RCLCPP_INFO(this->get_logger(), "Lidar[%s] 正在启动调度器...", lidar_config_.sn.c_str());
            alt_scheduler_->start([this](bool enable) {
                RCLCPP_INFO(this->get_logger(), "Lidar[%s] 调度器回调被触发: enable=%d", lidar_config_.sn.c_str(), enable);
                onAlternateSwitch(enable);
            });
            RCLCPP_INFO(this->get_logger(), "Lidar[%s] 调度器已启动", lidar_config_.sn.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Lidar[%s] 调度器未创建（可能未启用交替采集）", lidar_config_.sn.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "Lidar[%s] 节点已创建", lidar_config_.sn.c_str());
    }

    ~LidarNode()
    {
        // 停止交替采集调度器
        if (alt_scheduler_)
        {
            alt_scheduler_->stop();
        }
        RCLCPP_INFO(this->get_logger(), "Lidar[%s] 节点已销毁", lidar_config_.sn.c_str());
    }

protected:
    /**
     * @brief 采集数据接口（由子类实现具体逻辑）
     */
    virtual void collectData() = 0;

    

private:
    /**
     * @brief 采集句柄
     */
    bool collectHandle(){
        bool expected = false;
        if (!is_start_task_.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
        {
                RCLCPP_WARN(this->get_logger(), "Lidar[%s] 正在采集中: is_start_task_=%d", 
                           lidar_config_.sn.c_str(), expected);
                return false;
        }
        
        try {
            collectData();
            is_start_task_.store(false, std::memory_order_release);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Lidar[%s] collectData抛出异常: %s", lidar_config_.sn.c_str(), e.what());
            is_start_task_.store(false, std::memory_order_release);
            throw;  // 重新抛出，让RAII guard执行后再传播异常
        }
        
        return true;
    }

    /**
     * @brief 初始化节点相关参数
     */
    void initAlt()
    {
        // 读取交替采集参数
        enable_ = this->declare_parameter<bool>("alt.enable", false);
        if (enable_)
        {
            guard_ms_ = this->declare_parameter<int>("alt.guard_ms", 5000);
            on_ms_ = this->declare_parameter<int>("alt.on_ms", 60000);
            startup_delay_ms_ = this->declare_parameter<int>("alt.startup_delay_ms", 0);
            phase_count_ = this->declare_parameter<int>("alt.phase_count", 1);
            phase_index_ = this->declare_parameter<int>("alt.phase_index", 0);
            
            // 计算时间基准点（启动延迟后）
            auto now = std::chrono::steady_clock::now();
            auto delayed_epoch = now + std::chrono::milliseconds(startup_delay_ms_);
            int64_t epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                delayed_epoch.time_since_epoch()
            ).count();
            
            // 创建调度器
            alt_scheduler_ = std::make_unique<AlternateScheduler>(
                phase_count_,
                phase_index_,
                on_ms_,
                guard_ms_,
                epoch_ns
            );
            
            RCLCPP_INFO(this->get_logger(), 
                "Lidar[%s] 启动交替采集: phase=%d/%d, on=%dms, guard=%dms, delay=%dms",
                lidar_config_.sn.c_str(), phase_index_, phase_count_, 
                on_ms_, guard_ms_, startup_delay_ms_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Lidar[%s] 关闭交替采集模式", lidar_config_.sn.c_str());
        }
    }
    /**
     * @brief 初始化设备
     */
    void initDevices()
    {
        lidar_ = std::make_unique<lidar_base::LidarBase>(lidar_config_.lidar_ip, lidar_config_.local_ip, lidar_config_.sn);
        if (lidar_config_.type == LidarType::LIDAR_R1)
        {
            // 根据配置创建电机对象
            ByteOrder byte_order = motor_config_.byte_order;
            if (motor_config_.comm_type == "rtu")
            {
                auto modbus_rtu = std::make_shared<ModbusRTU>(
                    motor_config_.serial_port.value(),
                    motor_config_.baudrate.value(),
                    motor_config_.parity.value(),
                    motor_config_.data_bits.value(),
                    motor_config_.stop_bits.value()
                );
                motor_ = std::make_unique<motor_base::MotorBase>(modbus_rtu, motor_config_.slave_id, byte_order);
            }
            else if (motor_config_.comm_type == "tcp")
            {
                auto modbus_tcp = std::make_shared<ModbusTCP>(
                    motor_config_.motor_ip.value(),
                    motor_config_.network_port.value()
                );
                motor_ = std::make_unique<motor_base::MotorBase>(modbus_tcp, motor_config_.slave_id, byte_order);
            }
            else if (motor_config_.comm_type == "ttw")
            {
                auto ttw = std::make_shared<TransparentTransportWrapper>(
                    motor_config_.motor_ip.value(),
                    motor_config_.network_port.value()
                );
                motor_ = std::make_unique<motor_base::MotorBase>(ttw, motor_config_.slave_id, byte_order);
            }
            else
            {
                throw std::runtime_error("未知的电机通信类型: " + motor_config_.comm_type);
            }
        }
        else
        {
            motor_ = nullptr; // 80度雷达不需要电机
        }

        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // 初始化采集服务
        collect_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/collect_" + lidar_config_.sn,
            std::bind(&LidarNode::handleTrigger, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            callback_group_);
    }

    /**
     * @brief 处理采集请求服务回调
     */
    void handleTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            if (collectHandle()){
                response->success = true;
                response->message = "采集任务已完成";
                return;
            }
            else{
                response->success = false;
                response->message = "采集任务正在进行中，无法重复启动";
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Lidar[%s] 采集数据时发生异常：%s", lidar_config_.sn.c_str(), e.what());
            response->success = false;
            response->message = std::string("采集数据时发生异常: ") + e.what();
            return;
        }
    }
    /**
     * @brief 交替采集设备开关回调（子类可重写以实现自定义行为）
     * @param enable true=开启设备, false=关闭设备
     */
    void onAlternateSwitch(bool enable)
    {
        if (enable)
        {
            RCLCPP_INFO(this->get_logger(), "Lidar[%s] 交替采集: 退出保护期，开始采集", lidar_config_.sn.c_str());
            try{
                bool started = collectHandle();
                if (!started) {
                    RCLCPP_WARN(this->get_logger(), "Lidar[%s] 采集任务已在运行，跳过本次触发", lidar_config_.sn.c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Lidar[%s] 交替采集启动时发生异常：%s", lidar_config_.sn.c_str(), e.what());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Lidar[%s] 交替采集: 进入保护期，暂停采集", lidar_config_.sn.c_str());
        }
    }

protected:
    std::unique_ptr<lidar_base::LidarBase> lidar_;
    std::unique_ptr<motor_base::MotorBase> motor_;
    const LidarConfig lidar_config_;
    const MotorConfig motor_config_;
    const FilterConfig filter_config_;

private:
    std::atomic<bool> is_start_task_;                                    // 采集任务标志位
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr collect_service_; // 采集服务
    rclcpp::CallbackGroup::SharedPtr callback_group_;                    // 回调组

    // 交替采集相关
    bool enable_;                    // 是否启用交替采集
    int guard_ms_;                           // 保护时间（毫秒）
    int on_ms_;                             // 工作时间（毫秒）
    int startup_delay_ms_;                      // 启动延迟（毫秒）
    int phase_count_;                           // 总相位数
    int phase_index_;                           // 当前设备相位索引
    std::unique_ptr<AlternateScheduler> alt_scheduler_;
};