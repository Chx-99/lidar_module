#include "lidar_creator/lidar_creator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <vector>
#include <memory>
#include <chrono>

int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    auto lidar_creator = lidar_creator::LidarCreator("/opt/zwkj/configs/lidar_config.json", 3, 55000);
    auto lidar_nodes = lidar_creator.scanAndCreateAll();
    if (lidar_nodes.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("LidarCreator"), "没有可用的雷达节点");
        return 0;
    }

    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      lidar_nodes.size());

    // 将所有节点添加到执行器
    for (auto &node : lidar_nodes)
    {
        executor.add_node(node);
    }

    RCLCPP_INFO(rclcpp::get_logger("LidarCreator"),
                "启动执行器，共 %zu 个雷达节点", lidar_nodes.size());

    // 运行执行器（阻塞）
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
