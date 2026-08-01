// Copyright 2025 ackermann_vehicle_plugins contributors
// Licensed under the Apache License, Version 2.0

#ifndef EXPLICIT_ACKERMANN_PLUGIN_H
#define EXPLICIT_ACKERMANN_PLUGIN_H

// Gazebo 核心
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

// 运动学库
#include "ackermann_kinematics.h"

namespace ackermann_vehicle_plugins {

/// 显式阿克曼 Gazebo 插件
///
/// 与官方 gazebo_ros_ackermann_drive 的区别：
///   - 显式计算阿克曼转向角（基于轴距 + 轮距）
///   - PID 驱动每个关节到目标角度
///   - 所有几何参数从 SDF 读取，适用于任意阿克曼车辆
///
/// SDF 参数：
///   <wheelbase>       前后轴距 (m)
///   <track_width>     左右轮距 (m)
///   <wheel_radius>    车轮半径 (m)
///   <max_speed>       最大线速度 (m/s)
///   <max_angular_vel> 最大角速度 (rad/s)
///   <max_steer>       最大转向角 (rad)
///   <low_speed_threshold> 低速阈值 (m/s)
///   <steering_pid>    转向 PID (P I D)
///   <speed_pid>       速度 PID (P I D)
///   <pid_output_limit> PID 输出限幅
///   <odom_frame>      里程计坐标系名
///   <base_frame>      车体坐标系名
///   <odom_pub_rate>   里程计发布频率 (Hz)
///
/// 关节名约定（SDF/URDF 中必须一致）：
///   front_left_steering_joint
///   front_right_steering_joint
///   rear_left_wheel_joint
///   rear_right_wheel_joint
class ExplicitAckermannPlugin : public gazebo::ModelPlugin {
public:
    ExplicitAckermannPlugin();
    ~ExplicitAckermannPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
    // ── 回调 ──
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onUpdate(const gazebo::common::UpdateInfo& info);

    // ── 控制逻辑（委托给 AckermannKinematics） ──
    void updateSteering(double dt);
    void updateSpeed(double dt);
    void publishOdom(const gazebo::common::Time& sim_time);

    // ── Gazebo 物理组件 ──
    gazebo::physics::ModelPtr model_;
    gazebo::physics::JointPtr fl_steer_joint_;
    gazebo::physics::JointPtr fr_steer_joint_;
    gazebo::physics::JointPtr rl_wheel_joint_;
    gazebo::physics::JointPtr rr_wheel_joint_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time last_update_time_;
    gazebo::common::Time last_odom_pub_time_;

    // ── 运动学 ──
    AckermannKinematics kinematics_;

    // ── PID 控制器 ──
    gazebo::common::PID left_steering_pid_;
    gazebo::common::PID right_steering_pid_;
    gazebo::common::PID rear_left_pid_;
    gazebo::common::PID rear_right_pid_;
    double pid_output_limit_;

    // ── ROS 2 ──
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ── 控制状态 ──
    double target_linear_x_;
    double target_angular_z_;
    double target_steer_angle_;

    // ── 坐标系 ──
    std::string odom_frame_;
    std::string base_frame_;
    double odom_pub_interval_;  // 秒
};

}  // namespace ackermann_vehicle_plugins

#endif  // EXPLICIT_ACKERMANN_PLUGIN_H
