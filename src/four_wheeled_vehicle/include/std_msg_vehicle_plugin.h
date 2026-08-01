// # Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
// #                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
// #
// # Licensed under the Apache License, Version 2.0 (the "License");
// # you may not use this file except in compliance with the License.
// # You may obtain a copy of the License at
// #
// #     http://www.apache.org/licenses/LICENSE-2.0
// #
// # Unless required by applicable law or agreed to in writing, software
// # distributed under the License is distributed on an "AS IS" BASIS,
// # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// # See the License for the specific language governing permissions and
// # limitations under the License.




#ifndef STD_MSG_VEHICLE_PLUGIN_H
#define STD_MSG_VEHICLE_PLUGIN_H

// ROS 2标准消息头文件
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
// Gazebo核心头文件
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
// ROS 2核心头文件
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo {

// 新插件类：继承Gazebo ModelPlugin，使用ROS 2标准消息
class StdMsgVehiclePlugin : public ModelPlugin {
public:
    // 构造/析构函数
    StdMsgVehiclePlugin();
    ~StdMsgVehiclePlugin() override;

    // Gazebo插件标准加载接口（必须实现）
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
    // 1. 回调函数
    // cmd_vel订阅回调：解析标准Twist消息
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // Gazebo世界更新回调：每帧执行控制逻辑
    void OnUpdate(const common::UpdateInfo &_info);

    // 2. 控制逻辑函数
    // 转向控制：驱动前轮转向关节
    void UpdateSteering(double dt);
    // 速度控制：驱动后轮速度关节
    void UpdateSpeed(double dt);
    // 发布odom：输出标准里程计消息
    void PublishOdom(const common::Time &sim_time);

    common::Time last_odom_pub_time_;  // 新增：记录上一次发布odom的时间
    double odom_pub_interval_ = 0.1;  // 新增：odom发布间隔（单位：s，0.05s对应20Hz，可按需调整）

    // 3. Gazebo物理组件（车辆模型、关节、更新连接）
    physics::ModelPtr model_;                  // 车辆模型指针
    physics::JointPtr fl_steer_joint_;         // 左前轮转向关节
    physics::JointPtr fr_steer_joint_;         // 右前轮转向关节
    physics::JointPtr rl_wheel_joint_;         // 左后轮驱动关节
    physics::JointPtr rr_wheel_joint_;         // 右后轮驱动关节
    event::ConnectionPtr update_connection_;    // Gazebo更新连接
    common::Time last_update_time_;            // 上一帧更新时间

    // 4. PID控制器（转向2个、速度2个）
    common::PID left_steering_pid_;            // 左前轮转向PID
    common::PID right_steering_pid_;           // 右前轮转向PID
    common::PID rear_left_pid_;                // 左后轮速度PID
    common::PID rear_right_pid_;               // 右后轮速度PID

    // 5. ROS 2组件（节点、订阅/发布者）
    rclcpp::Node::SharedPtr ros_node_;         // ROS 2节点
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // 订阅cmd_vel
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;         // 发布odom

    // 6. 车辆控制量与参数
    double target_linear_x_;   // 目标线速度（x方向，对应cmd_vel.linear.x）
    double target_angular_z_;  // 目标角速度（z方向，对应cmd_vel.angular.z）
    double target_steering_angle_; // 由角速度推导的目标转向角
    double wheelbase_;         // 车辆轴距（m）
    double track_width_;       // 车辆轮距（m）
    double wheel_radius_;      // 车轮半径（m）
    double max_linear_x_;      // 最大线速度（m/s）
    double max_angular_z_;     // 最大角速度（rad/s）
    double max_steering_angle_; // 最大转向角（rad）
};

}  // namespace gazebo

#endif // STD_MSG_VEHICLE_PLUGIN_H