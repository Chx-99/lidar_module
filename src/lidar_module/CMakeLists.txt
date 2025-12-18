cmake_minimum_required(VERSION 3.8)
project(lidar_module)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 常规的编译选项
    add_compile_options(-Wall -Wextra -Wpedantic)
    # 固定为 Debug 模式
    add_compile_options(-g)  # 启用调试信息
endif()

# 通过 FetchContent 获取 ConcurrentQueue
include(FetchContent)
FetchContent_Declare(
  concurrentqueue
  GIT_REPOSITORY https://github.com/cameron314/concurrentqueue.git
  GIT_TAG master
)
FetchContent_MakeAvailable(concurrentqueue)

# 查找所需的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 主可执行文件
set(executable_name lidar_node)
add_executable(${executable_name} src/main.cpp)

# 目标包含目录和链接
target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${concurrentqueue_SOURCE_DIR}  # 添加 ConcurrentQueue 头文件路径
)

# 创建库
add_library(lidar_lib
  src/lidar_module/lidar.cc
  src/broadcast_listener/broadcast_listener.cc
)

# 为库也添加包含目录
target_include_directories(lidar_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${concurrentqueue_SOURCE_DIR}  # 添加 ConcurrentQueue 头文件路径
)

# 链接目标库
target_link_libraries(${executable_name}
  lidar_lib
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 设置编译选项
target_compile_options(${executable_name} PRIVATE -O3 -march=native)

# 声明依赖项
set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  PCL
  pcl_conversions
  ament_cmake
)

ament_target_dependencies(${executable_name} ${dependencies})

# 安装规则
install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})

# 启用测试支持
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()