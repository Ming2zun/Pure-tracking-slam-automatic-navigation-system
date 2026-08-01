// Copyright 2025 ackermann_vehicle_plugins contributors
// Licensed under the Apache License, Version 2.0

#include "explicit_ackermann_plugin.h"

#include <cstdio>
#include <gazebo/physics/Joint.hh>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ackermann_vehicle_plugins {

// ═══════════════════════════════════════════════════════════
//  构造 / 析构
// ═══════════════════════════════════════════════════════════

ExplicitAckermannPlugin::ExplicitAckermannPlugin()
    : kinematics_(AckermannParams{}),
      pid_output_limit_(5000.0),
      target_linear_x_(0.0),
      target_angular_z_(0.0),
      target_steer_angle_(0.0),
      odom_frame_("odom"),
      base_frame_("base_footprint"),
      odom_pub_interval_(0.1) {}

ExplicitAckermannPlugin::~ExplicitAckermannPlugin() {
    update_connection_.reset();
    ros_node_.reset();
}

// ═══════════════════════════════════════════════════════════
//  Load：从 SDF 读取所有参数
// ═══════════════════════════════════════════════════════════

void ExplicitAckermannPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
    model_ = model;

    if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("explicit_ackermann_plugin"),
                     "ROS 2 未初始化");
        return;
    }

    // ── 创建 ROS 2 节点 ──
    ros_node_ = rclcpp::Node::make_shared(
        "explicit_ackermann_controller",
        rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));

    // ── 从 SDF 读取运动学参数 ──
    AckermannParams kp;
    kp.wheelbase           = sdf->Get<double>("wheelbase", 3.0).first;
    kp.track_width         = sdf->Get<double>("track_width", 1.666).first;
    kp.wheel_radius        = sdf->Get<double>("wheel_radius", 0.3).first;
    kp.max_steer_angle     = sdf->Get<double>("max_steer", 0.699).first;
    kp.max_speed           = sdf->Get<double>("max_speed", 20.0).first;
    kp.max_angular_vel     = sdf->Get<double>("max_angular_vel", 1.0).first;
    kp.low_speed_threshold = sdf->Get<double>("low_speed_threshold", 0.01).first;
    kinematics_.setParams(kp);

    RCLCPP_INFO(ros_node_->get_logger(),
                "显式阿克曼插件加载: L=%.3f W=%.3f r=%.3f max_steer=%.3f",
                kp.wheelbase, kp.track_width, kp.wheel_radius, kp.max_steer_angle);

    // ── 从 SDF 读取 PID 参数 ──
    // 格式: <steering_pid>P I D</steering_pid>
    auto readPID = [&](const std::string& tag, double default_p, double default_i, double default_d) {
        if (sdf->HasElement(tag)) {
            std::string pid_str = sdf->Get<std::string>(tag);
            double p = default_p, i = default_i, d = default_d;
            if (sscanf(pid_str.c_str(), "%lf %lf %lf", &p, &i, &d) == 3) {
                return gazebo::common::PID(p, i, d);
            }
        }
        return gazebo::common::PID(default_p, default_i, default_d);
    };

    left_steering_pid_  = readPID("steering_pid", 2000.0, 0.0, 300.0);
    right_steering_pid_ = readPID("steering_pid", 2000.0, 0.0, 300.0);
    rear_left_pid_      = readPID("speed_pid", 1000.0, 0.0, 1.0);
    rear_right_pid_     = readPID("speed_pid", 1000.0, 0.0, 1.0);

    pid_output_limit_ = sdf->Get<double>("pid_output_limit", 5000.0).first;

    left_steering_pid_.SetCmdMin(-pid_output_limit_);
    left_steering_pid_.SetCmdMax(pid_output_limit_);
    right_steering_pid_.SetCmdMin(-pid_output_limit_);
    right_steering_pid_.SetCmdMax(pid_output_limit_);
    rear_left_pid_.SetCmdMin(-pid_output_limit_);
    rear_left_pid_.SetCmdMax(pid_output_limit_);
    rear_right_pid_.SetCmdMin(-pid_output_limit_);
    rear_right_pid_.SetCmdMax(pid_output_limit_);

    // ── 从 SDF 读取坐标系名 ──
    odom_frame_ = sdf->Get<std::string>("odom_frame", "odom").first;
    base_frame_ = sdf->Get<std::string>("base_frame", "base_footprint").first;

    // ── 里程计发布频率 ──
    double odom_pub_rate = sdf->Get<double>("odom_pub_rate", 10.0).first;
    odom_pub_interval_ = (odom_pub_rate > 0.0) ? (1.0 / odom_pub_rate) : 0.1;

    // ── 获取关节 ──
    fl_steer_joint_ = model_->GetJoint("front_left_steering_joint");
    fr_steer_joint_ = model_->GetJoint("front_right_steering_joint");
    rl_wheel_joint_ = model_->GetJoint("rear_left_wheel_joint");
    rr_wheel_joint_ = model_->GetJoint("rear_right_wheel_joint");

    if (!fl_steer_joint_ || !fr_steer_joint_ || !rl_wheel_joint_ || !rr_wheel_joint_) {
        RCLCPP_FATAL(ros_node_->get_logger(),
                     "未找到车辆关节，请检查 SDF/URDF 关节名："
                     "front_left_steering_joint, front_right_steering_joint, "
                     "rear_left_wheel_joint, rear_right_wheel_joint");
        return;
    }

    // ── 话题 ──
    std::string cmd_vel_topic = sdf->Get<std::string>("cmd_vel_topic", "cmd_vel").first;
    std::string odom_topic    = sdf->Get<std::string>("odom_topic", "odom").first;

    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10,
        std::bind(&ExplicitAckermannPlugin::cmdVelCallback, this, std::placeholders::_1));

    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    // ── Gazebo 更新回调 ──
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&ExplicitAckermannPlugin::onUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(ros_node_->get_logger(), "显式阿克曼插件就绪，订阅: %s, 发布: %s",
                cmd_vel_topic.c_str(), odom_topic.c_str());
}

// ═══════════════════════════════════════════════════════════
//  cmd_vel 回调：委托给运动学类计算目标转向角
// ═══════════════════════════════════════════════════════════

void ExplicitAckermannPlugin::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_linear_x_  = msg->linear.x;
    target_angular_z_ = msg->angular.z;

    // 运动学计算：vx, wz → 目标转向角 δ
    target_steer_angle_ = kinematics_.cmdVelToSteer(target_linear_x_, target_angular_z_);
}

// ═══════════════════════════════════════════════════════════
//  Gazebo 每帧更新
// ═══════════════════════════════════════════════════════════

void ExplicitAckermannPlugin::onUpdate(const gazebo::common::UpdateInfo& info) {
    rclcpp::spin_some(ros_node_);

    // 首帧初始化时间
    if (last_update_time_ == gazebo::common::Time(0)) {
        last_update_time_    = info.simTime;
        last_odom_pub_time_  = info.simTime;
        return;
    }

    double dt = (info.simTime - last_update_time_).Double();
    last_update_time_ = info.simTime;

    // PID 控制
    updateSteering(dt);
    updateSpeed(dt);

    // 里程计发布（按频率）
    if ((info.simTime - last_odom_pub_time_).Double() >= odom_pub_interval_) {
        publishOdom(info.simTime);
        last_odom_pub_time_ = info.simTime;
    }
}

// ═══════════════════════════════════════════════════════════
//  转向控制：委托给运动学类计算左右轮角度，PID 驱动
// ═══════════════════════════════════════════════════════════

void ExplicitAckermannPlugin::updateSteering(double dt) {
    // 运动学计算：δ → 左右前轮角度
    SteerAngles angles = kinematics_.computeWheelAngles(target_steer_angle_);

    double fl_current = fl_steer_joint_->Position(0);
    double fr_current = fr_steer_joint_->Position(0);

    double fl_force = left_steering_pid_.Update(fl_current - angles.left, dt);
    double fr_force = right_steering_pid_.Update(fr_current - angles.right, dt);

    fl_steer_joint_->SetForce(0, fl_force);
    fr_steer_joint_->SetForce(0, fr_force);
}

// ═══════════════════════════════════════════════════════════
//  速度控制：委托给运动学类计算后轮角速度，PID 驱动
// ═══════════════════════════════════════════════════════════

void ExplicitAckermannPlugin::updateSpeed(double dt) {
    // 运动学计算：vx → 后轮角速度
    double target_omega = kinematics_.linearToWheelOmega(target_linear_x_);

    double rl_current = rl_wheel_joint_->GetVelocity(0);
    double rr_current = rr_wheel_joint_->GetVelocity(0);

    double rl_force = rear_left_pid_.Update(rl_current - target_omega, dt);
    double rr_force = rear_right_pid_.Update(rr_current - target_omega, dt);

    rl_wheel_joint_->SetForce(0, rl_force);
    rr_wheel_joint_->SetForce(0, rr_force);
}

// ═══════════════════════════════════════════════════════════
//  里程计发布
// ═══════════════════════════════════════════════════════════

void ExplicitAckermannPlugin::publishOdom(const gazebo::common::Time& sim_time) {
    (void)sim_time;  // 使用 ros_node_->get_clock()->now() 获取仿真时间戳

    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = ros_node_->get_clock()->now();
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id  = base_frame_;

    // 位姿：从 Gazebo 世界坐标读取
    auto base_link = model_->GetLink(base_frame_);
    if (!base_link) {
        // 尝试 base_link（兼容不同 URDF 命名）
        base_link = model_->GetLink("base_link");
    }
    if (!base_link) return;

    auto pose = base_link->WorldPose();
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();
    msg.pose.pose.position.z = pose.Pos().Z();
    msg.pose.pose.orientation.x = pose.Rot().X();
    msg.pose.pose.orientation.y = pose.Rot().Y();
    msg.pose.pose.orientation.z = pose.Rot().Z();
    msg.pose.pose.orientation.w = pose.Rot().W();

    // 速度
    auto linear_vel  = base_link->WorldLinearVel();
    auto angular_vel = base_link->WorldAngularVel();
    msg.twist.twist.linear.x  = linear_vel.X();
    msg.twist.twist.angular.z = angular_vel.Z();

    odom_pub_->publish(msg);
}

// 注册为 Gazebo 模型插件
GZ_REGISTER_MODEL_PLUGIN(ExplicitAckermannPlugin)

}  // namespace ackermann_vehicle_plugins
