#pragma once

#include "lidar/lidar_base.h"
#include "lidar/lidar_scan.h"
#include "concurrentqueue.h"
#include "process/process_work.h"

#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

namespace lidar_manager
{
    using namespace lidar_base;
    using namespace lidar_scanner;
    using namespace base_frame;
    using namespace frame_tools;
    using namespace free_queue;
    using namespace process_worker;

    class LidarManager
    {
    public:
        LidarManager() = default;
        ~LidarManager() = default;

    private:
        boost::asio::io_context io_context;
        boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard;
        // 雷达扫描器
        LidarScanner scanner;
        // 雷达设备映射表
        std::unordered_map<std::string, std::unique_ptr<lidar_base::Lidar>> lidar_devices;
        // 数据处理器
        ProcessorThreadPool processor;
        // IO线程池
        std::vector<std::thread> io_threads;
        
    };

};