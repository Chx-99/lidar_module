#pragma once

#include "sensor/lidar/lidar_scan.hpp"
#include "lidar_80/lidar_01.hpp"
#include "lidar_180/lidar_r1.hpp"
#include "sensor/motor/base/comm_interface.hpp"
#include "sensor/motor/base/modbus_rtu.hpp"
#include "sensor/motor/base/modbus_tcp.hpp"
#include "sensor/motor/base/transparent_transport_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <stdexcept>
#include <string>

using json = nlohmann::json;

namespace lidar_creator
{
    
    /**
     * @brief 雷达生成器类
     *
     * 功能：
     * 1. 自动扫描网络中的所有雷达设备
     * 2. 根据配置文件或规则自动识别雷达类型
     * 3. 创建并管理所有雷达实例
     */
    class LidarCreator
    {
    public:
        /**
         * @brief 构造函数
         * @param scan_timeout 雷达扫描超时时间（秒）
         * @param scan_port 雷达广播端口
         */
        LidarCreator(int scan_timeout = 5, uint16_t scan_port = 55000, std::string config_file = "/opt/zwkj/configs/lidar_config.json")
            : scan_timeout_(scan_timeout), scan_port_(scan_port), config_file_(config_file)
        {
            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "雷达生成器初始化完成");
        }

        ~LidarCreator() = default;

        /**
         * @brief 自动扫描并创建所有雷达
         * @return 成功创建的雷达数量
         */
        std::vector<std::shared_ptr<rclcpp::Node>> scanAndCreateAll()
        {
            std::vector<std::shared_ptr<rclcpp::Node>> lidar_nodes;

            // 1. 扫描网络中的所有雷达
            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "开始扫描雷达设备...");
            lidar_scanner::LidarScanner scanner(scan_timeout_, scan_port_);
            auto scanned_lidars = scanner.searchLidar();

            if (scanned_lidars.empty())
            {
                throw std::runtime_error("未找到任何雷达设备");
            }

            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "扫描到 %zu 个雷达设备", scanned_lidars.size());

            // 2. 加载配置文件
            std::unordered_map<std::string, LidarConfig> config_map;
            if (!config_file_.empty())
            {
                config_map = loadConfigFile();
            }

            if (config_map.size() != scanned_lidars.size())
            {
                RCLCPP_WARN(rclcpp::get_logger("LidarCreator"),
                            "配置文件中的雷达数量 (%zu) 与扫描到的数量 (%zu) 不匹配",
                            config_map.size(), scanned_lidars.size());
            }

            // 3. 为每个扫描到的雷达创建实例
            for (const auto &[sn, info] : scanned_lidars)
            {
                try
                {
                    // 查找配置，如果没有则跳过
                    auto it = config_map.find(sn);
                    if (it == config_map.end())
                    {
                        RCLCPP_WARN(rclcpp::get_logger("LidarCreator"), "雷达 %s 未找到配置，跳过创建", sn.c_str());
                        continue;
                    }

                    // 更新配置中的IP地址（使用扫描到的实际IP）
                    LidarConfig config = it->second;
                    config.lidar_ip = info.lidar_ip;
                    config.local_ip = info.local_ip;

                    // 创建雷达实例
                    auto lidar_node = createLidarInstance(config);
                    lidar_nodes.push_back(std::move(lidar_node));
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("LidarCreator"), "创建雷达 %s 失败: %s", sn.c_str(), e.what());
                }
            }
            return lidar_nodes;
        }

    private:
        std::shared_ptr<rclcpp::Node> createLidarInstance(const LidarConfig &config)
        {
            if (config.type == LidarType::LIDAR_80)
            {
                RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "创建 Lidar01 实例，SN=%s, IP=%s", 
                           config.sn.c_str(), config.lidar_ip.c_str());
                return std::make_shared<Lidar01>(config);
            }
            else if (config.type == LidarType::LIDAR_180)
            {
                RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "创建 LidarR1 实例，SN=%s, IP=%s", 
                           config.sn.c_str(), config.lidar_ip.c_str());
                return std::make_shared<LidarR1>(config);
            }
            else
            {
                throw std::runtime_error("未知的雷达类型: SN=" + config.sn);
            }
        }
        /**
         * @brief 从配置文件加载雷达配置
         * @return 雷达配置映射表（键为SN）
         */
        std::unordered_map<std::string, LidarConfig> loadConfigFile()
        {
            std::unordered_map<std::string, LidarConfig> configs;

            try
            {
                std::ifstream file(config_file_);
                if (!file.is_open())
                {
                    throw std::runtime_error("无法打开配置文件: " + config_file_);
                }

                json j;
                file >> j;

                // 解析配置文件
                if (!j.contains("lidar80") && !j.contains("lidar180"))
                {
                    throw std::runtime_error("配置文件格式错误：缺少必要的雷达类型字段");
                }

                // 解析LIDAR_80配置
                if (j.contains("lidar80") && j["lidar80"].is_array())
                {
                    for (const auto &item : j["lidar80"])
                    {
                        LidarConfig config;
                        config.sn = item.value("sn", "");
                        config.type = LidarType::LIDAR_80;
                        config.lidar_ip = "";  // 将在扫描时填充
                        config.local_ip = "";  // 将在扫描时填充
                        config.is_save_pcd = item.value("is_save_pcd", false);
                        config.save_path = item.value("save_path", "/opt/zwkj/pcds/lidar80");
                        if (item.contains("lidar"))
                        {
                            const auto &lidar = item["lidar"];
                            config.accumulated_frames = lidar.value("accumulated_frames", 2500);
                        }
                        if (!config.sn.empty())
                        {
                            configs[config.sn] = config;
                        }
                    }
                }
                // 解析LIDAR_180配置
                if (j.contains("lidar180") && j["lidar180"].is_array())
                {
                    for (const auto &item : j["lidar180"])
                    {
                        LidarConfig config;
                        config.sn = item.value("sn", "");
                        config.type = LidarType::LIDAR_180;
                        config.lidar_ip = "";  // 将在扫描时填充
                        config.local_ip = "";  // 将在扫描时填充
                        config.is_save_pcd = item.value("is_save_pcd", false);
                        config.save_path = item.value("save_path", "/opt/zwkj/pcds/lidar180");
                        if (item.contains("motor"))
                        {
                            const auto &motor = item["motor"];
                            config.comm_type = motor.value("comm_type", "error");
                            config.slave_id = motor.value("slave_id", 1);
                            config.byte_order = motor.value("byte_order", "big_endian");
                            if (config.comm_type == "rtu")
                            {
                                config.serial_port = motor.value("serial_port", "/dev/ttyUSB0");
                                config.baudrate = motor.value("baudrate", 115200);
                                config.parity = motor.value("parity", "N")[0];
                                config.data_bits = motor.value("data_bits", 8);
                                config.stop_bits = motor.value("stop_bits", 1);
                            }
                            else if (config.comm_type == "tcp" || config.comm_type == "ttw")
                            {
                                config.network_port = motor.value("network_port", 502);
                                config.motor_ip = motor.value("motor_ip", "");
                            }
                            else
                            {
                                throw std::runtime_error("未知的电机通信类型: " + config.comm_type);
                            }
                        }
                        if (item.contains("lidar"))
                        {
                            const auto &lidar = item["lidar"];
                            config.accumulated_frames = lidar.value("accumulated_frames", 2500);
                            config.angle_segments = lidar.value("angle_segments", 6);
                            config.angle_y_deg = lidar.value("angle_y_deg", 37.5f);
                            config.voxel_size = lidar.value("voxel_size", 0.05f);
                        }
                        if (!config.sn.empty())
                        {
                            configs[config.sn] = config;
                        }
                    }
                }
                RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "成功加载配置文件，共 %zu 个雷达配置", configs.size());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LidarCreator"), "加载配置文件失败: %s", e.what());
            }
            return configs;
        }

        // 成员变量
        int scan_timeout_;         // 扫描超时时间
        unsigned short scan_port_; // 扫描端口
        std::string config_file_;  // 配置文件路径
    };

} // namespace lidar_creator
