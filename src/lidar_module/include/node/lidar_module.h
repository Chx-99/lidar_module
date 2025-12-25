#pragma once

#include "lidar/lidar_base.h"
#include "lidar/lidar_scan.h"
#include "concurrentqueue.h"
#include "process/data_analysis.h"

#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>


namespace lidar_manager{

    class LidarManager {
    public:
        LidarManager() = default;
        ~LidarManager() = default;
    private:
    };

};