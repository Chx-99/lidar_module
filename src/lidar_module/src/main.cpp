#include "lidar01/lidar01.hpp"
#include "sensor/lidar/lidar_scan.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <vector>
#include <memory>
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 扫描所有雷达设备
    lidar_scanner::LidarScanner scanner(2);
    auto lidars = scanner.searchLidar();

    if (lidars.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "未找到任何雷达设备");
        rclcpp::shutdown();
        return 1;
    }

    // 创建多线程执行器（线程数 = 雷达数量）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), lidars.size());

    // 创建所有 Lidar01 节点并添加到执行器
    std::vector<std::shared_ptr<Lidar01>> lidar_nodes;
    for (const auto &[sn, info] : lidars)
    {
        auto lidar = std::make_shared<Lidar01>(info.lidar_ip, info.local_ip, sn);
        executor.add_node(lidar);
        lidar_nodes.push_back(lidar);
        RCLCPP_INFO(rclcpp::get_logger("main"), "添加雷达: SN=%s, IP=%s", 
                    sn.c_str(), info.lidar_ip.c_str());
    }

   
    // 运行执行器（多线程处理所有节点的回调）
    RCLCPP_INFO(rclcpp::get_logger("main"), "启动执行器，共 %zu 个雷达节点", lidar_nodes.size());
    executor.spin();


    rclcpp::shutdown();
    return 0;
}
