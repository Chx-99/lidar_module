#ifndef LIDAR_H
#define LIDAR_H

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rmw/types.h>

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <nlohmann/adl_serializer.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "base/dataStruct.h"
#include "common_utils/fileExt.hpp"
#include "database_module/srv/database.hpp"
#include "lidar_module/srv/lidar_switch.hpp"
#include "rabbitmq_module/msg/custom_routing_msg.hpp"

class Lidar : public rclcpp::Node {
 public:
  /**
   * @brief 构造函数
   * @param ip Lidar的IP地址
   * @param sn Lidar的序列号
   * @param connect_ip 链接到雷达的IP地址
   * @param rate 发布数据的频率
   */
  explicit Lidar(const rclcpp::NodeOptions &options, const std::string &ip,
                 const std::string &sn, const std::string &connect_ip,
                 const std::string &type, int rate = 50);

  ~Lidar();

  bool switchLaser(bool);

  void setDataCallback(
      std::function<void(dataFrame<singleEchoRectangularData, 96>)> callback);

  void setIMUCallback(std::function<void(dataFrame<imuData, 1>)> callback);

 private:
  /**
   *  @brief 开启接收数据
   */
  void startReceive();

  /**
   *  @brief 处理接受的数据
   * @param bytes_received 接收的字节数
   */
  void handleReceive(std::size_t bytes_received);
  /**
   *  @brief 握手操作
   */
  void init();

  /**
   *  @brief 更新数据库雷达状态
   */
  void updateRadarStatusInDatabase(std::optional<bool> on, bool non_blocking);

  /**
   *  @brief 打开心跳
   */
  void startHeartBeat();

  /**
   *  @brief 检查心跳ack
   */
  void startHeartbeatChecker(int second = 3);

  /**
   *  @brief 处理心跳超时
   */
  void handleHeartBeatTimeout();

  /**
   *  @brief 打开imu数据
   */
  void receiveIMU();

  /**
   *  @brief 接受点云数据
   */
  void receiveData();

  /**
   *  @brief 检查交替采样租约
   */
  void checkAltLeases();

  /**
   *  @brief 发送指令
   * @param frame 数据帧
   * @param remote_endpoint 远程端点
   */
  template <typename T>
  void sendCommand(Frame<T> &frame,
                   const boost::asio::ip::udp::endpoint &remote_endpoint);

  void sendCommand(std::vector<uint8_t> &frame,
                   const boost::asio::ip::udp::endpoint &remote_endpoint);

  std::shared_ptr<std::vector<dataFrame<singleEchoRectangularData, 96>>>
  getPoints(size_t n);
  std::shared_ptr<dataFrame<imuData, 1>> getIMU();

  std::string ip_, sn_, connect_ip_, interface_name_, type_;
  int pointcloud_rate_;  //点云发布频率
  int imu_rate_;         //点云发布频率

  std::vector<unsigned short> ports;
  unsigned short cmd_port_;
  unsigned short data_port_;
  unsigned short imu_port_;

  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
      work_guard_;                                  // 保持 io_context 运行
  boost::asio::ip::udp::socket cmd_socket_;         // 用于发送命令
  boost::asio::ip::udp::socket recvIMU_socket_;     // 用于接收IMU数据
  boost::asio::ip::udp::socket recvDATA_socket_;    // 用于接收数据
  boost::asio::ip::udp::endpoint remote_endpoint_;  // 远程端点
  boost::asio::strand<boost::asio::io_context::executor_type>
      strand_;  // 用于确保异步操作的顺序执行

  std::deque<dataFrame<singleEchoRectangularData, 96>> point_packages;
  std::deque<dataFrame<imuData, 1>> imu_packages;

  boost::asio::steady_timer heart_timer_;        // 用于发送心跳
  boost::asio::steady_timer heartbeat_checker_;  // 用于处理心跳超时
  std::unordered_map<std::string, std::vector<uint8_t>>
      ackMap;  // 用于存储接收到的 ACK 数据
  std::mutex ack_mutex_, command_mutex_, pointcloud_data_mutex_,
      imu_data_mutex_;
  std::condition_variable ack_cv_, command_cv_, pointcloud_data_cv_,
      imu_data_cv_;

  std::atomic<bool> running_;            // 控制线程是否继续运行
  std::vector<std::thread> io_threads_;  // 用于运行 io_context 的线程

  std::vector<uint8_t> cmd_recv_buffer_;   // 用于存储接收到的CMD数据
  std::vector<uint8_t> data_recv_buffer_;  // 用于存储接收到的数据
  std::vector<uint8_t> imu_recv_buffer_;   // 用于存储接收到的IMU
  boost::asio::ip::udp::endpoint
      sender_endpoint_;  // 用于存储接收到数据的发送者端点

  std::function<void(dataFrame<singleEchoRectangularData, 96>)>
      dataCallback_;  // 数据回调函数
  std::function<void(dataFrame<imuData, 1>)> imuCallback_;  // IMU回调函数

  // ROS2 话题部分
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_publisher_;  // 点云发布器
  rclcpp::Publisher<rabbitmq_module::msg::CustomRoutingMsg>::SharedPtr
      warning_publisher_;  // 警告发布器
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      status_publisher_;  // 状态发布器
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      command_result_publisher_;  // 警告发布器
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      command_sub;  // 指令订阅
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
      imu_publisher_;  //日照港部分暂不实现

  //声明启停服务
  rclcpp::Service<lidar_module::srv::LidarSwitch>::SharedPtr
      lidar_switch_service_;

  // 数据库客户端
  rclcpp::Client<database_module::srv::Database>::SharedPtr database_client_;

  std::thread publich_pointcloud_thread_;  // 点云发布线程
  std::thread publich_imu_thread_;         // imu发布线程

  // === 心跳相关的新增成员 ===
  std::atomic<int64_t> last_hb_ack_ns_{
      0};  // 最近一次收到 HeartBeatACK 的时间点(steady ns)
  std::atomic<int> hb_miss_count_{0};  // 连续未收到次数
  int hb_consec_miss_limit_{3};        // 连续丢包阈值(建议 3 次)

  //===== 交替采集参数 =======
  rclcpp::TimerBase::SharedPtr timer_;
  bool alt_supported_ = false;  // 是否支持交替模式（来自参数 alt.enable）
  bool enable_ = false;  // 当前是否处于交替模式（运行时控制）
  int phase_count_{2};   // 相位总数
  int phase_index_{0};   // 当前相位索引
  int guard_ms_{100};    // 保护周期毫秒数
  int on_ms_{3000};      // 点亮周期毫秒数
  int period_ms_{1400};  // 总周期毫秒数
  int startup_delay_ms_{0};  // 启动延时毫秒数
  int64_t epoch_ns_{0};      // 交替采集时间基准 steady_ns

  std::atomic<bool> laser_on_{false};
  std::atomic<bool> switching_{false};
  std::chrono::steady_clock::time_point last_switch_tp_{};
  //判断是否在允许活动相位内
  bool in_active_window(int64_t now_ns) const;
  // 触发器
  void tick();
  //操作开关
  void switchOnce(bool want_on);

  std::mutex alt_mutex_;
  std::unordered_map<std::string, int64_t>
      alt_leases_;  // guid_key -> last_heartbeat_ns 服务使用的租约列表
  int alt_ref_cnt_ = 0;  // 当前交替采样引用计数 话题使用
  int alt_lease_timeout_ms_{10000};               // 心跳超时时间
  rclcpp::TimerBase::SharedPtr alt_lease_timer_;  // 定时检查超时
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr alt_srv_;

  void setAltStateLocked(bool enable);  // 持有 alt_mutex_ 时调用
  void handleAltService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> resp);
  //===== 交替采集参数 =======

  template <typename T>
  bool wait_and_parse_ack(const std::string &tag, T &dst,
                          int timeout_second = 3) {
    std::unique_lock<std::mutex> lock(ack_mutex_);
    if (!ack_cv_.wait_for(lock, std::chrono::seconds(timeout_second),
                          [this, &tag] { return ackMap.count(tag) > 0; })) {
      RCLCPP_ERROR(this->get_logger(), "[ACK] wait %s timeout", tag.c_str());
      return false;
    }
    // 现在还持有锁，原子地取出并删除
    auto it = ackMap.find(tag);
    if (it == ackMap.end()) return false;  // 双重保险
    const auto &buf = it->second;
    if (buf.size() < sizeof(T)) {
      RCLCPP_ERROR(this->get_logger(), "[ACK] %s too short: %zu < %zu",
                   tag.c_str(), buf.size(), sizeof(T));
      ackMap.erase(it);
      return false;
    }
    std::memcpy(&dst, buf.data(), sizeof(T));
    ackMap.erase(it);
    return true;
  }
};

#endif  // LIDAR_H
