cmake_minimum_required(VERSION 3.5)
project(four_wheeled_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- 车辆控制插件（保持你原有的）----
add_library(std_msg_vehicle_plugin SHARED
  src/std_msg_vehicle_plugin.cpp
)
target_include_directories(std_msg_vehicle_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(std_msg_vehicle_plugin
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(std_msg_vehicle_plugin
  gazebo_ros
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  nav_msgs
)

# ---- 新增：odom->map 变换发布节点 ----
add_executable(odom_mapTF src/odom_mapTF.cpp)
ament_target_dependencies(odom_mapTF
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---- 安装 ----
install(TARGETS
  std_msg_vehicle_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  odom_mapTF
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models launch worlds urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

# ---- 追加：odom->base_link 变换发布节点 ----
add_executable(odom_baselinkTF src/odom_baselinkTF.cpp)
ament_target_dependencies(odom_baselinkTF
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  odom_baselinkTF
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
