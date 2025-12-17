cmake_minimum_required(VERSION 3.8)
project(lidar_module_mwt)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 常规的编译选项
    add_compile_options(-Wall -Wextra -Wpedantic)
    # 固定为 Debug 模式
    add_compile_options(-g)  # 启用调试信息
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/LidarSwitch.srv"
)
# 可执行程序不要跟包名冲突！
add_executable(lidar_node_exec
        src/main.cpp
)
target_include_directories(lidar_node_exec PRIVATE
        ${CMAKE_INSTALL_PREFIX}/include
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
# 添加依赖
ament_target_dependencies(lidar_node_exec
        rclcpp
        std_msgs
        std_srvs
        sensor_msgs
        pcl_conversions
        nlohmann_json
)
# 接口类型绑定
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 链接头文件和库
target_link_libraries(lidar_node_exec
        ${cpp_typesupport_target}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
)


# 添加针对特定函数的性能测试可执行文件
add_executable(specific_performance_test
        test/specific_performance_test.cpp
)


target_include_directories(specific_performance_test PRIVATE
        ${CMAKE_INSTALL_PREFIX}/include
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)


ament_target_dependencies(specific_performance_test
        rclcpp
        std_msgs
        std_srvs
        sensor_msgs
        pcl_conversions
        nlohmann_json
)


target_link_libraries(specific_performance_test
        ${cpp_typesupport_target}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
)

# 安装可执行文件
install(TARGETS lidar_node_exec  specific_performance_test
        DESTINATION lib/${PROJECT_NAME}
)

# 包声明
ament_package()