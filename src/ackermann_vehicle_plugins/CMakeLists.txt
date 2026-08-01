cmake_minimum_required(VERSION 3.5)
project(ackermann_vehicle_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ── 运动学纯计算库（不依赖 ROS/Gazebo，可独立编译） ──
add_library(ackermann_kinematics SHARED
  src/ackermann_kinematics.cpp
)
target_include_directories(ackermann_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 链接数学库（Linux 下 M_PI 需要）
target_link_libraries(ackermann_kinematics m)

# ── Gazebo 显式阿克曼插件 ──
add_library(explicit_ackermann_plugin SHARED
  src/explicit_ackermann_plugin.cpp
)
target_include_directories(explicit_ackermann_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(explicit_ackermann_plugin
  ackermann_kinematics
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(explicit_ackermann_plugin
  gazebo_ros
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

# ── 单元测试 ──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ackermann_kinematics test/test_ackermann_kinematics.cpp)
  target_link_libraries(test_ackermann_kinematics ackermann_kinematics)
endif()

# ── 安装 ──
install(TARGETS ackermann_kinematics explicit_ackermann_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 导出头文件路径（供其他包 find_package 使用）
ament_export_include_directories(include)
ament_export_libraries(ackermann_kinematics explicit_ackermann_plugin)
ament_export_dependencies(gazebo_ros rclcpp geometry_msgs nav_msgs tf2)

ament_package()
