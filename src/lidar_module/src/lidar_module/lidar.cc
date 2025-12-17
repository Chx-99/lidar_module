// #include "lidar_module/lidar.h"

// #include <pcl/common/io.h>

// #include <array>
// #include <csignal>
// #include <exception>
// #include <memory>
// #include <mutex>
// #include <nlohmann/json_fwd.hpp>
// #include <rabbitmq_module/msg/detail/custom_routing_msg__struct.hpp>
// #include <rclcpp/qos.hpp>
// #include <std_msgs/msg/detail/string__struct.hpp>
// #include <stdexcept>
// #include <string>
// #include <thread>
// #include <vector>

// #include "base/convertStruct.h"
// #include "base/dataStruct.h"
// #include "base/tools.h"
// #include "common_utils/fileExt.hpp"
// #include "common_utils/time.hpp"
// #include "database_module/srv/database.hpp"

// extern bool DEBUG;

// static inline int64_t now_steady_ns() {
//   return std::chrono::duration_cast<std::chrono::nanoseconds>(
//              std::chrono::steady_clock::now().time_since_epoch())
//       .count();
// }

// static inline std::string guid_to_key(const uint8_t *guid) {
//   return std::string(reinterpret_cast<const char *>(guid),
//                      RMW_GID_STORAGE_SIZE);
// }

// Lidar::Lidar(const rclcpp::NodeOptions &options, const std::string &ip,
//              const std::string &sn, const std::string &connect_ip,
//              const std::string &type, int rate)
//     : rclcpp::Node("lidar_" + sn + "_node_" + getCurrentDateTimeString(false),
//                    options),
//       ip_(ip),
//       sn_(sn),
//       connect_ip_(connect_ip),
//       interface_name_(get_interface_name_from_ip(connect_ip)),
//       type_(type),
//       pointcloud_rate_(rate),
//       imu_rate_(200),
//       ports(getRandomAvailablePort(3)),
//       cmd_port_(ports[0]),
//       data_port_(ports[1]),
//       imu_port_(ports[2]),
//       work_guard_(boost::asio::make_work_guard(io_context_)),
//       cmd_socket_(io_context_,
//                   boost::asio::ip::udp::endpoint(
//                       boost::asio::ip::make_address_v4(connect_ip), cmd_port_)),
//       recvIMU_socket_(io_context_, boost::asio::ip::udp::endpoint(
//                                        boost::asio::ip::udp::v4(), imu_port_)),
//       recvDATA_socket_(io_context_,
//                        boost::asio::ip::udp::endpoint(
//                            boost::asio::ip::udp::v4(), data_port_)),
//       remote_endpoint_(boost::asio::ip::make_address_v4(ip_), 65000),
//       strand_(boost::asio::make_strand(io_context_)),
//       heart_timer_(io_context_),
//       heartbeat_checker_(io_context_),
//       running_(true) {
//   if (rate > 400) {
//     throw std::invalid_argument("频率不能高于400hz");
//   }

//   //获取交替采样相关参数
//   alt_supported_ = declare_parameter<bool>("alt.enable", false);
//   phase_count_ = declare_parameter<int>("alt.phase_count", 2);
//   phase_index_ = declare_parameter<int>("alt.phase_index", 0);
//   guard_ms_ = declare_parameter<int>("alt.guard_ms", 3000);
//   on_ms_ = declare_parameter<int>("alt.on_ms", 3000);
//   period_ms_ = declare_parameter<int>("alt.period_ms",
//                                       (on_ms_ + guard_ms_) * phase_count_);
//   startup_delay_ms_ = declare_parameter<int>("alt.startup_delay_ms", 0);
//   epoch_ns_ = declare_parameter<int64_t>("alt.epoch_steady_ns", 0);
//   alt_lease_timeout_ms_ = declare_parameter<int>("alt.lease_timeout_ms", 10000);

//   // 初始化缓冲区
//   cmd_recv_buffer_.clear();
//   cmd_recv_buffer_.reserve(2048);
//   data_recv_buffer_.clear();
//   data_recv_buffer_.reserve(2048);
//   imu_recv_buffer_.clear();
//   imu_recv_buffer_.reserve(2048);

//   RCLCPP_INFO(this->get_logger(),
//               "雷达 %s 初始化成功,指令端口: %d, 数据端口: %d, IMU端口: %d ,IP: "
//               "%s ,数据频率: %d",
//               sn_.c_str(), cmd_port_, data_port_, imu_port_, ip_.c_str(),
//               pointcloud_rate_);

//   cmd_socket_.set_option(boost::asio::socket_base::broadcast(true));
//   recvIMU_socket_.set_option(boost::asio::socket_base::reuse_address(true));
//   recvDATA_socket_.set_option(boost::asio::socket_base::reuse_address(true));

// #ifdef __linux__
//   if (!interface_name_.empty()) {
//     setsockopt(cmd_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE,
//                interface_name_.c_str(), interface_name_.size());
//     setsockopt(recvIMU_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE,
//                interface_name_.c_str(), interface_name_.size());
//     setsockopt(recvDATA_socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE,
//                interface_name_.c_str(), interface_name_.size());
//   }
// #endif
//   rclcpp::QoS pc_qos = rclcpp::SensorDataQoS().reliable();
//   rclcpp::QoS reliable_qos(rclcpp::KeepLast(100));
//   reliable_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // 可靠交付
//   reliable_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);  // 非持久化
//   reliable_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
//   reliable_qos.liveliness_lease_duration(std::chrono::seconds(5));
//   pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//       "/pointcloud_" + sn_, pc_qos);
//   imu_publisher_ =
//       this->create_publisher<sensor_msgs::msg::Imu>("/imu_" + sn_, pc_qos);
//   warning_publisher_ =
//       this->create_publisher<rabbitmq_module::msg::CustomRoutingMsg>(
//           "rabbitmq_task", reliable_qos);
//   status_publisher_ = this->create_publisher<std_msgs::msg::String>(
//       "lidar_status", reliable_qos);

//   alt_srv_ = this->create_service<std_srvs::srv::SetBool>(
//       "alt_control_" + sn_,
//       std::bind(&Lidar::handleAltService, this,
//                 std::placeholders::_1,  // rmw_request_id_t
//                 std::placeholders::_2,  // Request
//                 std::placeholders::_3   // Response
//                 ));

//   command_result_publisher_ = this->create_publisher<std_msgs::msg::String>(
//       "lidar_result", reliable_qos);
//   command_sub = this->create_subscription<std_msgs::msg::String>(
//       "lidar_task", reliable_qos,
//       [this](const std_msgs::msg::String::SharedPtr msg) {
//         try {
//           nlohmann::json command_data = nlohmann::json::parse(msg->data);

//           // 必须有 seq
//           if (!command_data.contains("seq") || !command_data["seq"].is_string())
//             return;
//           std::string seq = command_data["seq"].get<std::string>();

//           // 只处理发给自己的 或 广播 ALL 的命令
//           if (!command_data.contains("sn") || !command_data["sn"].is_string())
//             return;
//           std::string target_sn = command_data["sn"].get<std::string>();
//           if (target_sn != sn_ && target_sn != "ALL") return;

//           // 必须有 type
//           if (!command_data.contains("type") ||
//               !command_data["type"].is_string())
//             return;
//           std::string cmd_type = command_data["type"].get<std::string>();

//           // =========================================================
//           // 唯一保留的开关：交替采样开关（兼容 "switchAlt" 和 "switchData"）
//           // =========================================================
//           if (cmd_type == "switchAlt" || cmd_type == "switchData") {
//             // 必须有 enable: bool
//             if (!command_data.contains("enable") ||
//                 !command_data["enable"].is_boolean()) {
//               std_msgs::msg::String resultMsg;
//               nlohmann::json resultJson;
//               resultJson["success"] = false;
//               resultJson["message"] = "缺少布尔类型字段 enable";
//               resultJson["result"] =
//                   nlohmann::json{{"sn", sn_}, {"ip", ip_}}.dump();
//               resultJson["seq"] = seq;
//               resultMsg.data = resultJson.dump();
//               command_result_publisher_->publish(resultMsg);
//               return;
//             }

//             bool want_enable = command_data["enable"].get<bool>();

//             // 引用计数 key：优先用 client_id，没有就退回用 seq
//             std::string client_key;
//             if (command_data.contains("client_id") &&
//                 command_data["client_id"].is_string()) {
//               client_key = command_data["client_id"].get<std::string>();
//             } else {
//               client_key = seq;
//             }

//             bool ok = true;
//             std::string msg_text;

//             {
//               std::lock_guard<std::mutex> lk(alt_mutex_);

//               if (!alt_supported_) {
//                 ok = false;
//                 msg_text = "该雷达未启用交替采样功能";
//               } else {
//                 // 总引用数 = service 租约数 + topic 引用数
//                 int prev_total =
//                     static_cast<int>(alt_leases_.size()) + alt_ref_cnt_;

//                 if (want_enable) {
//                   // ===== topic 开启：引用计数 +1 =====
//                   ++alt_ref_cnt_;
//                   msg_text = "交替采样已开启（话题控制）";

//                 } else {
//                   // ===== topic 关闭：引用计数 -1，防止减成负数 =====
//                   if (alt_ref_cnt_ > 0) {
//                     --alt_ref_cnt_;
//                   } else {
//                     RCLCPP_WARN(
//                         this->get_logger(),
//                         "[%s] 交替采样关闭请求(话题)：引用计数已为 0, key=%s",
//                         sn_.c_str(), client_key.c_str());
//                   }
//                   msg_text = "交替采样已关闭（话题控制）";
//                 }

//                 int new_total =
//                     static_cast<int>(alt_leases_.size()) + alt_ref_cnt_;

//                 // 从 0 -> >0 : 打开交替采样
//                 if (prev_total == 0 && new_total > 0) {
//                   setAltStateLocked(true);
//                 }
//                 // 从 >0 -> 0 : 关闭交替采样
//                 if (prev_total > 0 && new_total == 0) {
//                   setAltStateLocked(false);
//                 }

//                 RCLCPP_INFO(
//                     this->get_logger(),
//                     "[%s] 交替采样状态变更(话题): enable=%d, topic_ref=%d, "
//                     "service_lease=%zu, total=%d",
//                     sn_.c_str(), want_enable ? 1 : 0, alt_ref_cnt_,
//                     alt_leases_.size(), new_total);
//               }
//             }  // 解锁 alt_mutex_

//             // 回结果到 lidar_result
//             std_msgs::msg::String resultMsg;
//             nlohmann::json resultJson;
//             resultJson["success"] = ok;
//             resultJson["message"] = msg_text;
//             resultJson["result"] =
//                 nlohmann::json{{"sn", sn_}, {"ip", ip_}}.dump();
//             resultJson["seq"] = seq;
//             resultMsg.data = resultJson.dump();
//             command_result_publisher_->publish(resultMsg);

//             return;  // 已处理
//           }

//           // 其他 type 暂时不处理，将来可以在这里继续扩展

//         } catch (const nlohmann::json::parse_error &e) {
//           RCLCPP_ERROR(this->get_logger(), "JSON 解析失败: %s", e.what());
//           return;
//         } catch (const std::exception &e) {
//           RCLCPP_ERROR(this->get_logger(), "命令处理异常: %s", e.what());
//           return;
//         }
//       });

//   // 初始化数据库客户端
//   database_client_ =
//       this->create_client<database_module::srv::Database>("database_service");

//   if (alt_supported_ && type_ != "R1") {
//     RCLCPP_INFO(this->get_logger(),
//                 "[%s] 已启用交替采样功能: 相位总数=%d, 当前相位索引=%d, "
//                 "点亮时长=%d ms, 周期=%d ms",
//                 sn_.c_str(), phase_count_, phase_index_, on_ms_, period_ms_);
//     alt_lease_timer_ = this->create_wall_timer(
//         std::chrono::seconds(1), std::bind(&Lidar::checkAltLeases, this));
//   }

//   if (type == "R1") {
//     lidar_switch_service_ =
//         this->create_service<lidar_module::srv::LidarSwitch>(
//             "lidar_switch",
//             [this](
//                 const std::shared_ptr<lidar_module::srv::LidarSwitch::Request>
//                     request,
//                 std::shared_ptr<lidar_module::srv::LidarSwitch::Response>
//                     response) -> void {
//               RCLCPP_INFO(this->get_logger(), "收到切换雷达状态请求");
//               bool success = true;
//               if (request->enable) {
//                 // 开激光
//                 Frame<Switch> sw{0x01};
//                 sendCommand(sw, remote_endpoint_);
//                 SwitchACK sw_ack{};
//                 if (!wait_and_parse_ack("switchACK", sw_ack) ||
//                     sw_ack.ret_code != 0) {
//                   RCLCPP_ERROR(this->get_logger(), "激光开启失败, ret=%d",
//                                sw_ack.ret_code);
//                   success = false;
//                 } else {
//                   RCLCPP_INFO(this->get_logger(), "激光开启成功");
//                 }
//                 // 设IMU频率
//                 Frame<setIMUFre> imu{0x01};
//                 sendCommand(imu, remote_endpoint_);
//                 setIMUFreACK imu_ack{};
//                 if (!wait_and_parse_ack("setIMUFreACK", imu_ack) ||
//                     imu_ack.ret_code != 0) {
//                   RCLCPP_ERROR(this->get_logger(), "设置IMU频率失败, ret=%d",
//                                imu_ack.ret_code);
//                   success = false;
//                 } else {
//                   RCLCPP_INFO(this->get_logger(), "设置IMU频率成功");
//                 }
//               } else {
//                 Frame<Switch> sw{0x00};
//                 sendCommand(sw, remote_endpoint_);

//                 SwitchACK sw_ack{};
//                 if (!wait_and_parse_ack("switchACK", sw_ack) ||
//                     sw_ack.ret_code != 0) {
//                   RCLCPP_ERROR(this->get_logger(), "激光关闭失败, ret=%d",
//                                sw_ack.ret_code);
//                   success = false;
//                 } else {
//                   RCLCPP_INFO(this->get_logger(), "激光关闭成功");
//                 }
//                 // 设IMU频率
//                 Frame<setIMUFre> imu{0x00};
//                 sendCommand(imu, remote_endpoint_);
//                 setIMUFreACK imu_ack{};
//                 if (!wait_and_parse_ack("setIMUFreACK", imu_ack) ||
//                     imu_ack.ret_code != 0) {
//                   RCLCPP_ERROR(this->get_logger(), "设置IMU频率失败, ret=%d",
//                                imu_ack.ret_code);
//                   success = false;
//                 } else {
//                   RCLCPP_INFO(this->get_logger(), "设置IMU频率成功");
//                 }
//               }
//               response->success = success;
//             });
//   }
//   startReceive();  // 启动接收数据
//   // 启动 4 个线程来运行 io_context_
//   for (int i = 0; i < 4; ++i) {
//     io_threads_.emplace_back([this]() { io_context_.run(); });
//   }
//   init();
// }

// Lidar::~Lidar() {
//   try {
//     running_ = false;
//     work_guard_.reset();
//     io_context_.stop();

//     if (publich_pointcloud_thread_.joinable())
//       publich_pointcloud_thread_.join();
//     if (publich_imu_thread_.joinable()) publich_imu_thread_.join();

//     for (auto &t : io_threads_) {
//       if (t.joinable()) {
//         t.join();
//       }
//     }

//     // 确保socket关闭时不会抛出异常
//     boost::system::error_code ec;
//     cmd_socket_.close(ec);
//     recvIMU_socket_.close(ec);
//     recvDATA_socket_.close(ec);
//   } catch (const std::exception &e) {
//     // 析构函数中处理异常，避免抛出
//     std::cerr << "Exception in destructor: " << e.what() << std::endl;
//   }
// }

// bool Lidar::switchLaser(bool status) {
//   Frame<Switch> sw(status ? 0x01 : 0x00);
//   sendCommand(sw, remote_endpoint_);
//   SwitchACK ack{};
//   if (!wait_and_parse_ack("switchACK", ack)) {
//     return false;
//   }
//   return ack.ret_code == 0;
// }

// bool Lidar::in_active_window(int64_t now_ns) const {
//   if (!alt_supported_ || !enable_) {
//     return false;
//   }

//   if (phase_count_ <= 0 || period_ms_ <= 0) {
//     return false;
//   }

//   const int64_t period_ns = static_cast<int64_t>(period_ms_) * 1'000'000LL;
//   const int64_t t_ns = now_ns - epoch_ns_;

//   // 還沒到 epoch 之前，一律关
//   if (t_ns < 0) {
//     return false;
//   }

//   // 當前時間在整個周期內的偏移
//   const int64_t mod = t_ns % period_ns;

//   // 每個相位的時間長度
//   const int64_t seg_ns = period_ns / phase_count_;
//   const int64_t phase_start = static_cast<int64_t>(phase_index_) * seg_ns;
//   const int64_t phase_end = phase_start + seg_ns;

//   // 不在自己的相位內，肯定关
//   if (mod < phase_start || mod >= phase_end) {
//     return false;
//   }

//   // 落在自己相位內，再細分為「点亮」和「保护区」
//   const int64_t offset_in_phase = mod - phase_start;
//   const int64_t on_ns = static_cast<int64_t>(on_ms_) * 1'000'000LL;

//   // 边界保护：如果 on_ms_ 未配置或比相位还长，就整个相位都亮
//   if (on_ns <= 0 || on_ns >= seg_ns) {
//     return true;
//   }

//   // 相位內 [0, on_ns) 为点亮区，其余时间为保护区，全黑
//   return offset_in_phase < on_ns;
// }

// void Lidar::tick() {
//   // 使用 steady_clock，避免系统时间跳变
//   const int64_t now_ns =
//       std::chrono::duration_cast<std::chrono::nanoseconds>(
//           std::chrono::steady_clock::now().time_since_epoch())
//           .count();
//   const bool want_on = in_active_window(now_ns);
//   switchOnce(want_on);
// }

// void Lidar::checkAltLeases() {
//   if (!alt_supported_) return;

//   std::lock_guard<std::mutex> lk(alt_mutex_);

//   if (alt_leases_.empty() && alt_ref_cnt_ == 0 && !enable_) {
//     // 已经没人使用且处于关闭状态，不用处理
//     return;
//   }

//   const int64_t now = now_steady_ns();
//   const int64_t timeout_ns =
//       static_cast<int64_t>(alt_lease_timeout_ms_) * 1'000'000LL;

//   bool removed_any = false;

//   for (auto it = alt_leases_.begin(); it != alt_leases_.end();) {
//     if (now - it->second > timeout_ns) {
//       it = alt_leases_.erase(it);
//       removed_any = true;
//     } else {
//       ++it;
//     }
//   }

//   if (removed_any) {
//     RCLCPP_WARN(get_logger(),
//                 "[%s] 交替采样租约超时, 剩余 service 客户端数量=%zu, "
//                 "topic 引用数=%d",
//                 sn_.c_str(), alt_leases_.size(), alt_ref_cnt_);
//   }

//   // 只有当 service + topic 都没人了，才自动关掉
//   if (alt_leases_.empty() && alt_ref_cnt_ == 0 && enable_) {
//     setAltStateLocked(false);
//     RCLCPP_WARN(get_logger(),
//                 "[%s] 所有交替采样租约已过期且无话题引用, 自动关闭交替采样",
//                 sn_.c_str());
//   }
// }

// void Lidar::setAltStateLocked(bool enable) {
//   if (enable == enable_) return;

//   if (enable) {
//     enable_ = true;

//     if (phase_count_ > 1) {
//       // ===== 多相位：按照时间相位交替采样 =====
//       // 先明确关一次，之后由 tick() 按相位开关
//       switchOnce(false);

//       if (!timer_) {
//         auto first_fire =
//             std::chrono::milliseconds(std::max(10, startup_delay_ms_));
//         timer_ = this->create_wall_timer(first_fire, [this]() {
//           // 第一次触发后，改成固定周期 tick
//           timer_->cancel();
//           timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
//                                            std::bind(&Lidar::tick, this));
//         });
//       }

//       RCLCPP_INFO(get_logger(),
//                   "[%s] AltSampling ENABLED (multi-phase, phase_count=%d)",
//                   sn_.c_str(), phase_count_);

//     } else {
//       // ===== 单相位：无交替，只要有租约就长开 =====
//       // 不再创建 / 使用 tick 定时器，直接常亮
//       if (timer_) {
//         timer_->cancel();
//         timer_.reset();
//       }

//       switchOnce(true);  // 直接打开激光

//       RCLCPP_INFO(get_logger(),
//                   "[%s] AltSampling ENABLED (single-phase constant ON)",
//                   sn_.c_str());
//     }

//   } else {
//     // ===== 关闭：不管单相位/多相位，统一逻辑 =====
//     enable_ = false;

//     // 停掉交替定时器（如果之前有）
//     if (timer_) {
//       timer_->cancel();
//       timer_.reset();
//     }

//     // 确保激光关闭
//     switchOnce(false);

//     RCLCPP_INFO(get_logger(), "[%s] AltSampling DISABLED & laser OFF",
//                 sn_.c_str());
//   }
// }

// void Lidar::handleAltService(
//     const std::shared_ptr<rmw_request_id_t> request_header,
//     const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
//     std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
//   std::lock_guard<std::mutex> lk(alt_mutex_);

//   if (!alt_supported_) {
//     resp->success = false;
//     resp->message = "该雷达未启用交替采样功能";
//     RCLCPP_WARN(get_logger(),
//                 "[%s] 收到 alt_control 请求, 但 alt_supported_ = false",
//                 sn_.c_str());
//     return;
//   }

//   const uint8_t *caller_guid = request_header->writer_guid;
//   const std::string key = guid_to_key(caller_guid);
//   const int64_t now = now_steady_ns();

//   if (req->data) {
//     // ===== SetBool(true): 申请 / 刷新租约 =====
//     bool was_empty = alt_leases_.empty();
//     alt_leases_[key] = now;  // 新 client 或刷新心跳

//     if (was_empty) {
//       // 第一个 client 进来：开启交替采样
//       setAltStateLocked(true);
//       RCLCPP_INFO(get_logger(),
//                   "[%s] alt_control: 首个客户端申请租约, 已开启交替采样",
//                   sn_.c_str());
//     } else {
//       RCLCPP_DEBUG(get_logger(),
//                    "[%s] alt_control: 刷新租约, 当前客户端数量=%zu",
//                    sn_.c_str(), alt_leases_.size());
//     }

//     resp->success = true;
//     resp->message = "交替采样租约已申请/刷新";

//   } else {
//     // ===== SetBool(false): 主动释放租约 =====
//     auto it = alt_leases_.find(key);
//     if (it != alt_leases_.end()) {
//       alt_leases_.erase(it);
//       RCLCPP_INFO(
//           get_logger(),
//           "[%s] alt_control: 客户端释放租约, 剩余 service 客户端数量=%zu, "
//           "topic 引用数=%d",
//           sn_.c_str(), alt_leases_.size(), alt_ref_cnt_);
//     } else {
//       RCLCPP_WARN(get_logger(),
//                   "[%s] alt_control: 请求释放租约, 但未找到对应客户端",
//                   sn_.c_str());
//     }

//     int total = static_cast<int>(alt_leases_.size()) + alt_ref_cnt_;
//     if (total == 0 && enable_) {
//       setAltStateLocked(false);
//       RCLCPP_INFO(
//           get_logger(),
//           "[%s] alt_control: 所有租约与话题引用均已释放, 交替采样已关闭",
//           sn_.c_str());
//     }

//     resp->success = true;
//     resp->message = "交替采样租约已释放";
//   }
// }

// void Lidar::switchOnce(bool want_on) {
//   bool old = laser_on_.load(std::memory_order_relaxed);
//   if (old == want_on) return;

//   // 防抖: 最小保持时间，避免边界抖动
//   constexpr auto kMinHold = std::chrono::milliseconds(50);
//   auto now = std::chrono::steady_clock::now();
//   if (now - last_switch_tp_ < kMinHold) return;

//   // 避免并发切换
//   bool expected = false;
//   if (!switching_.compare_exchange_strong(expected, true)) return;

//   bool ok = switchLaser(want_on);
//   if (ok) {
//     laser_on_.store(want_on, std::memory_order_relaxed);
//     last_switch_tp_ = now;
//     RCLCPP_INFO(get_logger(), "[%s] 激光状态切换为: %s", sn_.c_str(),
//                 want_on ? "开启" : "关闭");
//   } else {
//     RCLCPP_WARN(get_logger(), "[%s] 调用 switchLaser(%d) 失败或超时",
//                 sn_.c_str(), int(want_on));
//   }
//   switching_.store(false, std::memory_order_relaxed);
// }

// // 接受CMD数据
// void Lidar::startReceive() {
//   // 创建动态缓冲区适配器
//   auto dynamicBuf = boost::asio::dynamic_buffer(cmd_recv_buffer_);
//   // 为本次接收预留 1024 字节的空间
//   auto mutableBuffer = dynamicBuf.prepare(1024);

//   cmd_socket_.async_receive_from(
//       mutableBuffer, sender_endpoint_,
//       boost::asio::bind_executor(strand_, [this](boost::system::error_code ec,
//                                                  std::size_t bytes_received) {
//         if (!ec && running_) {
//           auto dynamicBuf = boost::asio::dynamic_buffer(cmd_recv_buffer_);
//           // 告诉动态缓冲区，这次接收操作获得了 bytes_received 字节
//           dynamicBuf.commit(bytes_received);
//           // 调用原有的处理函数处理接收到的数据
//           handleReceive(bytes_received);
//           // 数据处理完后，清空缓冲区，以便下次接收
//           dynamicBuf.consume(dynamicBuf.size());
//           // 继续下一次接收
//           startReceive();
//         }
//       }));
// }

// void Lidar::handleReceive(std::size_t bytes_received) {
//   try {
//     std::string tag = TAG_MAP.at({cmd_recv_buffer_[CMD_TYPE_INDEX],
//                                   cmd_recv_buffer_[CMD_SET_INDEX],
//                                   cmd_recv_buffer_[CMD_ID_INDEX]});
//     // 切分获取数据域
//     std::vector<uint8_t> data_v = GET_DATA_FIELD(
//         reinterpret_cast<char *>(cmd_recv_buffer_.data()), bytes_received);

//     if (tag == "heartBeatACK") {
//       // === 心跳 ACK：刷新时间戳，清零连续丢包计数 ===
//       heartBeatACK ack;
//       if (data_v.size() >= sizeof(heartBeatACK)) {
//         std::memcpy(&ack, data_v.data(), sizeof(heartBeatACK));
//       }
//       last_hb_ack_ns_.store(now_steady_ns(), std::memory_order_relaxed);
//       hb_miss_count_.store(0, std::memory_order_relaxed);

//       if (DEBUG) {
//         std::cout << "Received: heartBeatACK" << std::endl;
//         std::cout << ack << std::endl;
//       }
//     } else {
//       {
//         std::lock_guard<std::mutex> lock(ack_mutex_);
//         ackMap[tag] = data_v;
//       }

//       if (DEBUG) {
//         std::cout << "Received: " << tag << std::endl;
//         if (tag == "handShakeACK") {
//           handShakeACK ack;
//           if (data_v.size() >= sizeof(handShakeACK)) {
//             std::memcpy(&ack, data_v.data(), sizeof(handShakeACK));
//             std::cout << ack << std::endl;
//           }
//         } else if (tag == "errorMsg") {
//           ExceptionMsg msg;
//           if (data_v.size() >= sizeof(ExceptionMsg)) {
//             std::memcpy(&msg, data_v.data(), sizeof(ExceptionMsg));
//             auto errJson = parseLidarStatusToJson(msg.status_code);
//             nlohmann::json warnMsg;
//             warnMsg["ip"] = ip_;
//             warnMsg["sn"] = sn_;
//             warnMsg["msg"] = errJson;
//             rabbitmq_module::msg::CustomRoutingMsg rosMsg;
//             rosMsg.content = warnMsg.dump();
//             warning_publisher_->publish(rosMsg);
//           }
//         }
//       }
//       ack_cv_.notify_one();
//     }
//   } catch (std::out_of_range &) {
//     std::cerr << "Error: Unknown tag received." << std::endl;
//   }
// }

// void Lidar::setDataCallback(
//     std::function<void(dataFrame<singleEchoRectangularData, 96>)> callback) {
//   dataCallback_ = callback;
// }

// void Lidar::setIMUCallback(
//     std::function<void(dataFrame<imuData, 1>)> callback) {
//   imuCallback_ = callback;
// }

// void Lidar::init() {
//   Frame<handShake> handshake{getIpVector(connect_ip_), data_port_, cmd_port_,
//                              imu_port_};
//   std::cout << "输入ip:" << ip_ << std::endl;
//   boost::asio::ip::udp::endpoint init_remote_endpoint(
//       boost::asio::ip::make_address_v4(ip_), 65000);
//   sendCommand(handshake, init_remote_endpoint);
//   handShakeACK ack{};
//   if (wait_and_parse_ack("handShakeACK", ack)) {
//     std::cout << ack << std::endl;

//     if (ack.ret_code == 0) {
//       std::cout << "链接成功" << std::endl;
//       last_hb_ack_ns_.store(now_steady_ns(), std::memory_order_relaxed);
//       hb_miss_count_.store(0, std::memory_order_relaxed);

//       // 更新数据库雷达状态为已连接
//       std::thread([this]() {
//         updateRadarStatusInDatabase(true, false);
//       }).detach();

//       startHeartBeat();  // 启动心跳机制
//       startHeartbeatChecker(5);
//       receiveData();  // 启动数据接收
//       receiveIMU();

//       //普通雷达默认开启点云
//       if (type_ == "normal" && !alt_supported_) {
//         Frame<Switch> open{0x01};
//         sendCommand(open, remote_endpoint_);
//         SwitchACK ack{};
//         if (!wait_and_parse_ack("switchACK", ack, 5)) {
//           throw std::runtime_error("激光开启失败");
//         }
//       }
//       publich_pointcloud_thread_ = std::thread{[this] {
//         std::cout << "进入pointcloud发送线程" << std::endl;
//         //控制rate来发送数据
//         //需要提取出 rate个数据 组装成ros数据 然后发送
//         rclcpp::Rate rate(pointcloud_rate_);  // 每秒发布 10 次
//         int target_packets = 240000 / (96 * pointcloud_rate_);
//         RCLCPP_INFO(this->get_logger(), "每秒需要发送的点云包数: %d",
//                     target_packets);
//         while (rclcpp::ok(this->get_node_base_interface()->get_context()) &&
//                running_) {
//           {
//             // RCLCPP_INFO(this->get_logger(), "等待数据包...");
//             std::unique_lock<std::mutex> lock(pointcloud_data_mutex_);
//             pointcloud_data_cv_.wait(lock, [this, target_packets] {
//               return !running_ || point_packages.size() >= target_packets;
//             });
//             if (!running_) break;
//             auto points = this->getPoints(target_packets);
//             if (points == nullptr) continue;
//             RCLCPP_INFO(this->get_logger(),
//                         "雷达 %s 发布 %zu 个点云包, %zu 个点.", sn_.c_str(),
//                         points->size(), points->size() * 96);
//             auto rosMsg = convertToPointCloud2(this->get_clock(), points, sn_);
//             pointcloud_publisher_->publish(rosMsg);
//           }
//           rate.sleep();
//         }
//         std::cout << "发送线程结束" << std::endl;
//       }};
//       if (type_ == "R1") {
//         publich_imu_thread_ = std::thread{[this] {
//           std::cout << "进入imu发送线程" << std::endl;
//           //控制rate来发送数据
//           //需要提取出 rate个数据 组装成ros数据 然后发送
//           rclcpp::Rate rate(imu_rate_);  // 每秒发布 200 次
//           while (rclcpp::ok(this->get_node_base_interface()->get_context()) &&
//                  running_) {
//             {
//               // RCLCPP_INFO(this->get_logger(), "等待数据包...");
//               std::unique_lock<std::mutex> lock(imu_data_mutex_);
//               imu_data_cv_.wait(lock, [this] {
//                 return !running_ || imu_packages.size() > 0;
//               });
//               if (!running_) break;
//               auto imu_data = this->getIMU();
//               // RCLCPP_INFO(this->get_logger(), "雷达 %s 发布 imu数据包",
//               //             sn_.c_str());
//               if (imu_data == nullptr) continue;
//               auto rosMsg = convertToIMU(this->get_clock(), imu_data, sn_);
//               imu_publisher_->publish(rosMsg);
//             }
//             rate.sleep();
//           }
//           std::cout << "发送线程结束" << std::endl;
//         }};
//       }
//     }
//   } else {
//     handleHeartBeatTimeout();
//   }
// }

// void Lidar::startHeartBeat() {
//   heart_timer_.expires_after(std::chrono::seconds(1));
//   heart_timer_.async_wait(boost::asio::bind_executor(
//       strand_, [this](const boost::system::error_code &ec) {
//         if (!ec && running_) {
//           Frame<heartBeat> heartbeat;
//           sendCommand(heartbeat, remote_endpoint_);
//           startHeartBeat();  // 下一轮
//         }
//       }));
// }

// void Lidar::startHeartbeatChecker(int timeout_sec) {
//   heartbeat_checker_.expires_after(std::chrono::seconds(timeout_sec));
//   heartbeat_checker_.async_wait(boost::asio::bind_executor(
//       strand_, [this, timeout_sec](const boost::system::error_code &ec) {
//         if (ec || !running_) return;

//         const int64_t last = last_hb_ack_ns_.load(std::memory_order_relaxed);
//         const int64_t delta_ns = now_steady_ns() - last;
//         const int64_t limit_ns =
//             static_cast<int64_t>(timeout_sec) * 1'000'000'000LL;

//         // 若从未收到过ACK（冷启动阶段），给一次宽限：不判死，继续等待
//         if (last == 0) {
//           startHeartbeatChecker(timeout_sec);
//           return;
//         }

//         if (delta_ns > limit_ns) {
//           int miss = hb_miss_count_.fetch_add(1) + 1;
//           RCLCPP_WARN(get_logger(), "心跳未确认，delta=%.3fs, 连续miss=%d/%d",
//                       delta_ns / 1e9, miss, hb_consec_miss_limit_);
//           if (miss >= hb_consec_miss_limit_) {
//             handleHeartBeatTimeout();
//             return;
//           }
//         } else {
//           hb_miss_count_.store(0, std::memory_order_relaxed);
//         }

//         startHeartbeatChecker(timeout_sec);  // 继续下一轮检查
//       }));
// }

// void Lidar::handleHeartBeatTimeout() {
//   RCLCPP_ERROR(this->get_logger(), "心跳 / 握手超时! 未收到ACK响应");

//   running_.store(false, std::memory_order_relaxed);
//   // 发布状态消息
//   try {
//     std_msgs::msg::String msg;
//     msg.data = sn_;
//     status_publisher_->publish(msg);
//     RCLCPP_ERROR(this->get_logger(), "通知管理节点,雷达 %s 异常.", sn_.c_str());
//   } catch (const std::exception &e) {
//     RCLCPP_ERROR(this->get_logger(), "发布状态消息失败: %s", e.what());
//   }

//   boost::system::error_code ec;
//   heart_timer_.cancel(ec);
//   heartbeat_checker_.cancel(ec);
// }
// void Lidar::receiveIMU() {
//   // 创建动态缓冲区适配器
//   auto dynamicBuf = boost::asio::dynamic_buffer(imu_recv_buffer_);
//   // 为本次接收预留 1024 字节的空间
//   auto mutableBuffer = dynamicBuf.prepare(1024);
//   recvIMU_socket_.async_receive_from(
//       mutableBuffer, sender_endpoint_,
//       boost::asio::bind_executor(strand_, [this](boost::system::error_code ec,
//                                                  std::size_t bytes_received) {
//         if (!ec && running_) {
//           auto dynamicBuf = boost::asio::dynamic_buffer(imu_recv_buffer_);
//           // 告诉动态缓冲区，这次接收操作获得了 bytes_received 字节
//           dynamicBuf.commit(bytes_received);
//           dataFrame<imuData, 1> imuData;
//           std::memcpy(&imuData, imu_recv_buffer_.data(), bytes_received);

//           if (imuCallback_) {
//             imuCallback_(imuData);
//           }
//           if (DEBUG) {
//             std::cout << imuData << std::endl;
//           }
//           {
//             std::unique_lock<std::mutex> lock{imu_data_mutex_};
//             imu_packages.push_back(std::move(imuData));
//             imu_data_cv_.notify_one();
//           }
//           // 数据处理完后，清空缓冲区，以便下次接收
//           dynamicBuf.consume(dynamicBuf.size());
//           // 继续下一次接收
//           receiveIMU();
//         }
//       }));
// }

// void Lidar::receiveData() {
//   static thread_local std::vector<char> raw_buffer(4096);

//   recvDATA_socket_.async_receive_from(
//       boost::asio::buffer(raw_buffer), sender_endpoint_,
//       boost::asio::bind_executor(strand_, [this](boost::system::error_code ec,
//                                                  std::size_t bytes_received) {
//         try {
//           if (!ec && running_) {
//             const size_t expected_size =
//                 sizeof(dataFrame<singleEchoRectangularData, 96>);

//             // 添加边界检查
//             if (bytes_received > raw_buffer.size()) {
//               std::cerr << "[ERROR] Buffer overflow risk: received "
//                         << bytes_received << " bytes, buffer size "
//                         << raw_buffer.size() << std::endl;
//               receiveData();
//               return;
//             }

//             if (bytes_received != expected_size) {
//               std::cerr << "[WARN] Invalid packet size: " << bytes_received
//                         << " (expected " << expected_size << ")\n";
//               receiveData();  // 不污染、不存历史，直接放弃
//               return;
//             }

//             dataFrame<singleEchoRectangularData, 96> data;
//             std::memcpy(&data, raw_buffer.data(), expected_size);

//             if (dataCallback_) {
//               dataCallback_(data);
//             }

//             if (DEBUG) {
//               std::cout << "[DEBUG] Packet OK, size: " << bytes_received
//                         << "\n";
//               std::cout << data << std::endl;
//             }
//             {
//               std::unique_lock<std::mutex> lock{pointcloud_data_mutex_};
//               point_packages.push_back(std::move(data));
//               pointcloud_data_cv_.notify_one();
//             }
//           } else if (ec) {
//             std::cerr << "[ERROR] receiveData(): " << ec.message() << std::endl;
//           }
//         } catch (const std::exception &e) {
//           std::cerr << "[EXCEPTION] receiveData(): " << e.what() << std::endl;
//         }
//         receiveData();  // 继续监听
//       }));
// }

// std::shared_ptr<std::vector<dataFrame<singleEchoRectangularData, 96>>>
// Lidar::getPoints(size_t n) {
//   if (point_packages.size() < n) {
//     return nullptr;
//   }

//   std::shared_ptr<std::vector<dataFrame<singleEchoRectangularData, 96>>> data =
//       std::make_shared<std::vector<dataFrame<singleEchoRectangularData, 96>>>();
//   for (size_t i = 0; i < n; ++i) {
//     data->push_back(point_packages.front());
//     point_packages.pop_front();
//   }
//   return data;
// }
// std::shared_ptr<dataFrame<imuData, 1>> Lidar::getIMU() {
//   if (imu_packages.empty()) {
//     return nullptr;
//   }
//   // 使用 move 语义避免复制
//   auto data =
//       std::make_shared<dataFrame<imuData, 1>>(std::move(imu_packages.front()));
//   imu_packages.pop_front();
//   return data;
// }
// // ====================================================================================================

// void Lidar::sendCommand(
//     std::vector<uint8_t> &frame,
//     const boost::asio::ip::udp::endpoint &remote_endpoint_) {
//   if (!cmd_socket_.is_open() || !running_) {
//     RCLCPP_WARN(this->get_logger(), "Socket未打开或设备未运行");
//     return;
//   }
//   try {
//     size_t bytes_sent =
//         cmd_socket_.send_to(boost::asio::buffer(frame), remote_endpoint_);
//     if (bytes_sent != frame.size()) {
//       RCLCPP_WARN(this->get_logger(), "数据发送不完整: %zu/%zu", bytes_sent,
//                   frame.size());
//     }
//   } catch (const boost::system::system_error &e) {
//     RCLCPP_ERROR(this->get_logger(), "发送命令失败: %s", e.what());
//   } catch (const std::exception &e) {
//     RCLCPP_ERROR(this->get_logger(), "发送命令异常: %s", e.what());
//   }
// }

// template <typename T>
// void Lidar::sendCommand(
//     Frame<T> &frame, const boost::asio::ip::udp::endpoint &remote_endpoint_) {
//   if (!cmd_socket_.is_open() || !running_) {
//     return;
//   }
//   try {
//     std::vector<uint8_t> data = frame.serialize();
//     cmd_socket_.send_to(boost::asio::buffer(data), remote_endpoint_);
//   } catch (const std::exception &e) {
//     std::cerr << "Send command failed: " << e.what() << std::endl;
//   }
// }

// void Lidar::updateRadarStatusInDatabase(std::optional<bool> on,
//                                         bool non_blocking) {
//   if (!database_client_) {
//     RCLCPP_WARN(get_logger(), "数据库客户端未初始化");
//     return;
//   }

//   // non_blocking=true 时：不等待服务（避免阻塞回调）
//   if (!non_blocking) {
//     if (!database_client_->wait_for_service(std::chrono::seconds(5))) {
//       RCLCPP_WARN(get_logger(), "数据库服务不可用，雷达[%s]状态更新失败",
//                   sn_.c_str());
//       return;
//     }
//   }

//   // 生成 SQL：
//   // - 如果 on 有值：根据开关写 '1' / '0'
//   // - 如果 on 没值：保持原来“握手成功后标记为开启”的逻辑（写 '1'）
//   std::string is_open = on.has_value() ? (on.value() ? "1" : "0") : "1";
//   std::string sql = "update zw_radar set isOpen = '" + is_open +
//                     "' where radarNo like '" + sn_ + "%'";

//   auto request = std::make_shared<database_module::srv::Database::Request>();
//   request->command.push_back(sql);
//   request->is_sync = !non_blocking;  // 非阻塞时置为 false
//   request->seq = "RADAR_ISOPEN";

//   // 总是用异步发送；同步/异步的语义由 is_sync 决定
//   database_client_->async_send_request(
//       request,
//       [this, is_open](
//           rclcpp::Client<database_module::srv::Database>::SharedFuture future) {
//         try {
//           auto response = future.get();
//           if (response->success) {
//             RCLCPP_INFO(get_logger(), "雷达[%s] isOpen=%s 更新成功: %s",
//                         sn_.c_str(), is_open.c_str(),
//                         response->result_json.c_str());
//           } else {
//             RCLCPP_ERROR(get_logger(), "雷达[%s] isOpen=%s 更新失败: %s",
//                          sn_.c_str(), is_open.c_str(),
//                          response->result_json.c_str());
//           }
//         } catch (const std::exception &e) {
//           RCLCPP_ERROR(get_logger(), "雷达[%s] 状态更新回调异常: %s",
//                        sn_.c_str(), e.what());
//         }
//       });
// }
