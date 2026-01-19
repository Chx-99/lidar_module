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
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(nlohmann_json REQUIRED)

# 使用 pkg-config 查找 libmodbus
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# PCL相关的编译选项（必须与PCL库编译选项保持一致）
# 不要添加任何可能改变ABI的Eigen宏定义

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${LIBMODBUS_INCLUDE_DIRS}
)

# 链接目录
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 主可执行文件
set(executable_name lidar_node)
add_executable(${executable_name} 
  src/main.cpp
)

# 目标包含目录和链接
target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${concurrentqueue_SOURCE_DIR}  # 添加 ConcurrentQueue 头文件路径
)

# 链接目标库
target_link_libraries(${executable_name}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBMODBUS_LIBRARIES}
  atomic  # C++20 atomic shared_ptr support
)

# 设置编译选项（调试模式保持简单，避免与PCL库不兼容）
target_compile_options(${executable_name} PRIVATE -g -O0 -fno-omit-frame-pointer)

# 声明依赖项
set(dependencies
  rclcpp
  sensor_msgs
  std_srvs
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