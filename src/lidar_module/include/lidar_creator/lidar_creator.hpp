#pragma once

#include "sensor/lidar/lidar_scan.hpp"
#include "lidar_80/lidar_01.hpp"
#include "lidar_180/lidar_r1.hpp"
#include "sensor/motor/base/plc_device_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <stdexcept>
#include <string>
#include <optional>
#include <algorithm>

using json = nlohmann::json;

namespace lidar_creator
{
    /**
     * @brief 时间片分配配置
     */
    struct AlternateConfig
    {
        bool enable = false;             // 是否启用交替采集
        int on_ms = 60000;               // 工作时间（毫秒）
        int guard_ms = 5000;             // 保护时间（毫秒）
        int startup_delay_ms = 0;        // 启动延迟（毫秒）
    };

    /**
     * @brief 雷达组配置（同组雷达共享时间片）
     */
    struct LidarGroupConfig
    {
        std::vector<LidarConfig> lidar_configs;    // 雷达配置列表
        std::vector<MotorConfig> motor_configs;    // 电机配置列表（与雷达一一对应）
        std::vector<FilterConfig> filter_configs;  // 滤波器配置列表（与雷达一一对应）
        int phase_index = 0;                       // 时间片索引
    };

    /**
     * @brief 雷达生成器类
     *
     * 功能：
     * 1. 自动扫描网络中的所有雷达设备
     * 2. 根据配置文件加载分组配置
     * 3. 为每组分配时间片参数
     * 4. 创建并管理所有雷达实例
     */
    class LidarCreator
    {
    public:
        /**
         * @brief 构造函数
         * @param config_file 配置文件路径
         * @param scan_timeout 雷达扫描超时时间（秒）
         * @param scan_port 雷达广播端口
         */
        explicit LidarCreator(std::string config_file = "/opt/zwkj/configs/lidar_config.json",
                             int scan_timeout = 5, 
                             uint16_t scan_port = 55000)
            : config_file_(std::move(config_file)), 
              scan_timeout_(scan_timeout), 
              scan_port_(scan_port)
        {
            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), 
                       "雷达生成器初始化完成，配置文件: %s", config_file_.c_str());
        }

        ~LidarCreator() = default;

        // 禁用拷贝和移动
        LidarCreator(const LidarCreator&) = delete;
        LidarCreator& operator=(const LidarCreator&) = delete;

        /**
         * @brief 扫描并创建所有雷达实例
         * @return 创建的雷达节点列表
         * @throws std::runtime_error 如果扫描或创建失败
         */
        [[nodiscard]] std::vector<std::shared_ptr<rclcpp::Node>> scanAndCreateAll()
        {
            // 1. 扫描网络中的所有雷达
            auto scanned_lidars = scanLidars();

            // 2. 加载配置文件
            auto [groups, alt_config] = loadConfiguration();

            // 3. 验证扫描结果与配置匹配
            validateScannedLidars(scanned_lidars, groups);

            // 4. 更新配置中的IP地址（使用扫描到的实际IP）
            updateIPAddresses(groups, scanned_lidars);

            // 5. 创建所有雷达实例
            return createLidarInstances(groups, alt_config);
        }

    private:
        // ==================== 扫描相关 ====================

        /**
         * @brief 扫描网络中的雷达设备
         * @return 扫描到的雷达信息映射表（键为SN）
         * @throws std::runtime_error 如果未找到任何雷达
         */
        [[nodiscard]] std::unordered_map<std::string, lidar_scanner::LidarInfo> scanLidars()
        {
            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), "开始扫描雷达设备...");
            
            lidar_scanner::LidarScanner scanner(scan_timeout_, scan_port_);
            auto scanned_lidars = scanner.searchLidar();

            if (scanned_lidars.empty())
            {
                throw std::runtime_error("未找到任何雷达设备");
            }

            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), 
                       "扫描到 %zu 个雷达设备", scanned_lidars.size());

            return scanned_lidars;
        }

        // ==================== 配置加载 ====================

        /**
         * @brief 加载配置文件
         * @return (雷达组列表, 交替采集配置)
         * @throws std::runtime_error 如果配置文件格式错误
         */
        [[nodiscard]] std::pair<std::vector<LidarGroupConfig>, AlternateConfig> loadConfiguration()
        {
            std::ifstream file(config_file_);
            if (!file.is_open())
            {
                throw std::runtime_error("无法打开配置文件: " + config_file_);
            }

            json j;
            file >> j;

            // 加载交替采集配置
            AlternateConfig alt_config = parseAlternateConfig(j);

            // 加载雷达组配置
            std::vector<LidarGroupConfig> groups;
            
            if (j.contains("lidar80") && j["lidar80"].is_array())
            {
                parseLidarGroups(j["lidar80"], LidarType::LIDAR_80, groups);
            }

            if (j.contains("lidar180") && j["lidar180"].is_array())
            {
                parseLidarGroups(j["lidar180"], LidarType::LIDAR_R1, groups);
            }

            if (groups.empty())
            {
                throw std::runtime_error("配置文件中没有有效的雷达配置");
            }

            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"), 
                       "成功加载配置文件，共 %zu 个雷达组", groups.size());

            return {groups, alt_config};
        }

        /**
         * @brief 解析交替采集配置
         */
        [[nodiscard]] AlternateConfig parseAlternateConfig(const json& j) const
        {
            AlternateConfig config;

            if (j.contains("alt") && j["alt"].is_object())
            {
                const auto& alt = j["alt"];
                config.enable = alt.value("enable", false);
                config.on_ms = alt.value("on_ms", 60000);
                config.guard_ms = alt.value("guard_ms", 5000);
                config.startup_delay_ms = alt.value("startup_delay_ms", 0);

                RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                           "交替采集配置: enable=%s, on=%dms, guard=%dms, delay=%dms",
                           config.enable ? "true" : "false", 
                           config.on_ms, config.guard_ms, config.startup_delay_ms);
            }

            return config;
        }

        /**
         * @brief 解析雷达组配置
         * @param groups_json JSON数组（每个元素是一个组的雷达数组）
         * @param type 雷达类型
         * @param output_groups 输出的雷达组列表
         */
        void parseLidarGroups(const json& groups_json, 
                             LidarType type,
                             std::vector<LidarGroupConfig>& output_groups)
        {
            for (const auto& group_json : groups_json)
            {
                if (!group_json.is_array())
                {
                    RCLCPP_WARN(rclcpp::get_logger("LidarCreator"), 
                               "跳过非数组格式的雷达组配置");
                    continue;
                }

                LidarGroupConfig group;
                group.phase_index = static_cast<int>(output_groups.size());

                for (const auto& item : group_json)
                {
                    try
                    {
                        if (type == LidarType::LIDAR_80)
                        {
                            parseLidar80Config(item, group);
                        }
                        else // LIDAR_R1
                        {
                            parseLidar180Config(item, group);
                        }
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("LidarCreator"),
                                    "解析雷达配置失败: %s", e.what());
                    }
                }

                if (!group.lidar_configs.empty())
                {
                    // 收集组内所有雷达的SN
                    std::string sn_list;
                    for (size_t i = 0; i < group.lidar_configs.size(); ++i)
                    {
                        if (i > 0) sn_list += ", ";
                        sn_list += group.lidar_configs[i].sn;
                    }
                    
                    output_groups.push_back(std::move(group));
                    RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                               "加载雷达组 %d 配置: [%s] (共享时间片 %d)",
                               group.phase_index, sn_list.c_str(), group.phase_index);
                }
            }
        }

        /**
         * @brief 解析80度雷达配置
         */
        void parseLidar80Config(const json& item, LidarGroupConfig& group)
        {
            LidarConfig lidar_cfg;
            lidar_cfg.sn = item.value("sn", "");
            lidar_cfg.type = LidarType::LIDAR_80;
            lidar_cfg.lidar_ip = "";  // 扫描时填充
            lidar_cfg.local_ip = "";  // 扫描时填充

            // 解析雷达参数
            if (item.contains("lidar") && item["lidar"].is_object())
            {
                const auto& lidar = item["lidar"];
                lidar_cfg.accumulated_frames = lidar.value("accumulated_frames", 2500);
            }

            // 解析滤波器配置
            FilterConfig filter_cfg = parseFilterConfig(item);

            // 80度雷达不需要电机
            MotorConfig motor_cfg;

            if (!lidar_cfg.sn.empty())
            {
                group.lidar_configs.push_back(std::move(lidar_cfg));
                group.motor_configs.push_back(std::move(motor_cfg));
                group.filter_configs.push_back(std::move(filter_cfg));
            }
        }

        /**
         * @brief 解析180度雷达配置
         */
        void parseLidar180Config(const json& item, LidarGroupConfig& group)
        {
            LidarConfig lidar_cfg;
            lidar_cfg.sn = item.value("sn", "");
            lidar_cfg.type = LidarType::LIDAR_R1;
            lidar_cfg.lidar_ip = "";  // 扫描时填充
            lidar_cfg.local_ip = "";  // 扫描时填充

            // 解析雷达参数
            if (item.contains("lidar") && item["lidar"].is_object())
            {
                const auto& lidar = item["lidar"];
                lidar_cfg.accumulated_frames = lidar.value("accumulated_frames", 2500);
                lidar_cfg.angle_segments = lidar.value("angle_segments", 6);
                lidar_cfg.motor_offset_y_angle = lidar.value("motor_offset_y_angle", 37.5f);
                lidar_cfg.motor_bias_z_distance = lidar.value("motor_bias_z_distance", 0.0f);
            }

            // 解析电机配置
            MotorConfig motor_cfg = parseMotorConfig(item);

            // 解析滤波器配置
            FilterConfig filter_cfg = parseFilterConfig(item);

            if (!lidar_cfg.sn.empty())
            {
                group.lidar_configs.push_back(std::move(lidar_cfg));
                group.motor_configs.push_back(std::move(motor_cfg));
                group.filter_configs.push_back(std::move(filter_cfg));
            }
        }

        /**
         * @brief 解析电机配置
         */
        [[nodiscard]] MotorConfig parseMotorConfig(const json& item) const
        {
            MotorConfig motor_cfg;

            if (item.contains("motor") && item["motor"].is_object())
            {
                const auto& motor = item["motor"];
                motor_cfg.comm_type = motor.value("comm_type", "error");
                motor_cfg.slave_id = motor.value("slave_id", 1);

                std::string byte_order_str = motor.value("byte_order", "big_endian");
                motor_cfg.byte_order = (byte_order_str == "big_endian") 
                                      ? ByteOrder::BigEndian 
                                      : ByteOrder::LittleEndian;

                if (motor_cfg.comm_type == "rtu")
                {
                    motor_cfg.serial_port = motor.value("serial_port", "/dev/ttyUSB0");
                    motor_cfg.baudrate = motor.value("baudrate", 115200);
                    
                    std::string parity_str = motor.value("parity", "N");
                    motor_cfg.parity = parity_str.empty() ? 'N' : parity_str[0];
                    
                    motor_cfg.data_bits = motor.value("data_bits", 8);
                    motor_cfg.stop_bits = motor.value("stop_bits", 1);
                }
                else if (motor_cfg.comm_type == "tcp" || motor_cfg.comm_type == "ttw")
                {
                    motor_cfg.motor_ip = motor.value("motor_ip", "");
                    motor_cfg.network_port = motor.value("network_port", 502);
                }
                else
                {
                    throw std::runtime_error("未知的电机通信类型: " + motor_cfg.comm_type);
                }
            }

            return motor_cfg;
        }

        /**
         * @brief 解析滤波器配置
         */
        [[nodiscard]] FilterConfig parseFilterConfig(const json& item) const
        {
            FilterConfig filter_cfg;

            if (item.contains("filter") && item["filter"].is_object())
            {
                const auto& filter = item["filter"];

                // 解析变换矩阵
                if (filter.contains("transform_to_world") && filter["transform_to_world"].is_array())
                {
                    const auto& transform = filter["transform_to_world"];
                    if (transform.size() == 4)
                    {
                        for (size_t i = 0; i < 4; ++i)
                        {
                            if (transform[i].is_array() && transform[i].size() == 4)
                            {
                                for (size_t j = 0; j < 4; ++j)
                                {
                                    filter_cfg.transform_to_world(i, j) = transform[i][j].get<float>();
                                }
                            }
                        }
                    }
                }

                // 解析体素大小
                filter_cfg.voxel_size = filter.value("voxel_size", 0.05f);

                // 解析直通滤波器
                if (filter.contains("passthrough") && filter["passthrough"].is_array())
                {
                    for (const auto& pass : filter["passthrough"])
                    {
                        PassThroughFilter pass_filter;
                        pass_filter.field_name = pass.value("field_name", "z");
                        
                        if (pass.contains("limits") && pass["limits"].is_array() && pass["limits"].size() == 2)
                        {
                            pass_filter.limits.first = pass["limits"][0].get<float>();
                            pass_filter.limits.second = pass["limits"][1].get<float>();
                        }
                        
                        filter_cfg.passthrough.push_back(std::move(pass_filter));
                    }
                }
            }

            return filter_cfg;
        }

        // ==================== 验证与更新 ====================

        /**
         * @brief 验证扫描结果与配置匹配
         */
        void validateScannedLidars(
            const std::unordered_map<std::string, lidar_scanner::LidarInfo>& scanned_lidars,
            const std::vector<LidarGroupConfig>& groups) const
        {
            size_t config_count = 0;
            for (const auto& group : groups)
            {
                config_count += group.lidar_configs.size();
            }

            if (config_count != scanned_lidars.size())
            {
                RCLCPP_WARN(rclcpp::get_logger("LidarCreator"),
                           "配置文件中的雷达数量 (%zu) 与扫描到的数量 (%zu) 不匹配",
                           config_count, scanned_lidars.size());
            }
        }

        /**
         * @brief 更新配置中的IP地址
         */
        void updateIPAddresses(
            std::vector<LidarGroupConfig>& groups,
            const std::unordered_map<std::string, lidar_scanner::LidarInfo>& scanned_lidars)
        {
            size_t matched_count = 0;
            size_t total_count = 0;
            
            for (auto& group : groups)
            {
                for (auto& lidar_cfg : group.lidar_configs)
                {
                    total_count++;
                    auto it = scanned_lidars.find(lidar_cfg.sn);
                    if (it != scanned_lidars.end())
                    {
                        lidar_cfg.lidar_ip = it->second.lidar_ip;
                        lidar_cfg.local_ip = it->second.local_ip;
                        matched_count++;
                        
                        RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                                   "匹配雷达 %s: %s -> %s",
                                   lidar_cfg.sn.c_str(),
                                   it->second.local_ip.c_str(),
                                   it->second.lidar_ip.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger("LidarCreator"),
                                   "配置的雷达 %s 未在扫描结果中找到，将不会创建",
                                   lidar_cfg.sn.c_str());
                    }
                }
            }
            
            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                       "IP地址更新完成: 匹配 %zu/%zu 个雷达",
                       matched_count, total_count);
        }

        // ==================== 实例创建 ====================

        /**
         * @brief 创建所有雷达实例
         */
        [[nodiscard]] std::vector<std::shared_ptr<rclcpp::Node>> createLidarInstances(
            const std::vector<LidarGroupConfig>& groups,
            const AlternateConfig& alt_config)
        {
            std::vector<std::shared_ptr<rclcpp::Node>> lidar_nodes;

            for (const auto& group : groups)
            {
                size_t group_created_count = 0;
                std::string created_sn_list;
                
                // 计算组内的总相位数（有多少个雷达就有多少个相位）
                const int group_phase_count = static_cast<int>(group.lidar_configs.size());
                
                for (size_t i = 0; i < group.lidar_configs.size(); ++i)
                {
                    const auto& lidar_cfg = group.lidar_configs[i];
                    
                    // 检查雷达是否在扫描结果中找到（IP地址是否已更新）
                    if (lidar_cfg.lidar_ip.empty() || lidar_cfg.local_ip.empty())
                    {
                        RCLCPP_WARN(rclcpp::get_logger("LidarCreator"),
                                   "雷达 %s 未在扫描结果中找到，跳过创建",
                                   lidar_cfg.sn.c_str());
                        continue;
                    }
                    
                    try
                    {
                        // 组内相位索引：第i个雷达的相位索引就是i (从0开始)
                        const int lidar_phase_index = static_cast<int>(i);
                        
                        auto node = createSingleLidar(
                            lidar_cfg,
                            group.motor_configs[i],
                            group.filter_configs[i],
                            alt_config,
                            lidar_phase_index,     // 组内相位索引 (0-based)
                            group_phase_count      // 组内总相位数
                        );

                        if (node)
                        {
                            lidar_nodes.push_back(std::move(node));
                            group_created_count++;
                            
                            if (!created_sn_list.empty()) created_sn_list += ", ";
                            created_sn_list += lidar_cfg.sn;
                        }
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("LidarCreator"),
                                    "创建雷达 %s 失败: %s",
                                    lidar_cfg.sn.c_str(), e.what());
                    }
                }
                
                // 输出每个组的创建统计
                if (group_created_count > 0)
                {
                    RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                               "组 %d 成功创建 %zu 个雷达: [%s] (组内共 %d 个时间片，雷达按顺序分配)",
                               group.phase_index, group_created_count, 
                               created_sn_list.c_str(), group_phase_count);
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("LidarCreator"),
                               "组 %d 没有成功创建任何雷达", group.phase_index);
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                       "========== 总计成功创建 %zu 个雷达实例 ==========", lidar_nodes.size());

            return lidar_nodes;
        }

        /**
         * @brief 创建单个雷达实例
         */
        [[nodiscard]] std::shared_ptr<rclcpp::Node> createSingleLidar(
            const LidarConfig& lidar_cfg,
            const MotorConfig& motor_cfg,
            const FilterConfig& filter_cfg,
            const AlternateConfig& alt_config,
            int phase_index,
            int total_phases)
        {
            // 创建节点选项并设置参数
            rclcpp::NodeOptions options;
            
            if (alt_config.enable)
            {
                options.append_parameter_override("alt.enable", true);
                options.append_parameter_override("alt.on_ms", alt_config.on_ms);
                options.append_parameter_override("alt.guard_ms", alt_config.guard_ms);
                options.append_parameter_override("alt.startup_delay_ms", alt_config.startup_delay_ms);
                options.append_parameter_override("alt.phase_index", phase_index);
                options.append_parameter_override("alt.phase_count", total_phases);

                RCLCPP_DEBUG(rclcpp::get_logger("LidarCreator"),
                           "为雷达 %s 配置时间片: 相位=%d, 总相位数=%d",
                           lidar_cfg.sn.c_str(), phase_index, total_phases);
            }

            // 根据类型创建雷达实例
            if (lidar_cfg.type == LidarType::LIDAR_80)
            {
                RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                           "创建 Lidar01 实例 [%s]: %s -> %s",
                           lidar_cfg.sn.c_str(),
                           lidar_cfg.local_ip.c_str(),
                           lidar_cfg.lidar_ip.c_str());

                return std::make_shared<Lidar01>(options, lidar_cfg, motor_cfg, filter_cfg);
            }
            else if (lidar_cfg.type == LidarType::LIDAR_R1)
            {
                RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                           "创建 LidarR1 实例 [%s]: %s -> %s",
                           lidar_cfg.sn.c_str(),
                           lidar_cfg.local_ip.c_str(),
                           lidar_cfg.lidar_ip.c_str());

                return std::make_shared<LidarR1>(options, lidar_cfg, motor_cfg, filter_cfg);
            }
            else
            {
                throw std::runtime_error("未知的雷达类型: SN=" + lidar_cfg.sn);
            }
        }

        // ==================== 成员变量 ====================

        const std::string config_file_;  // 配置文件路径
        const int scan_timeout_;         // 扫描超时时间（秒）
        const uint16_t scan_port_;       // 扫描端口
    };

} // namespace lidar_creator
