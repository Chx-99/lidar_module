#include "lidar/lidar_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_base::LidarNode>());
    rclcpp::shutdown();
    return 0;
}
