#include "std_msg_vehicle_plugin.h"
#include <gazebo/physics/Joint.hh>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo {

StdMsgVehiclePlugin::StdMsgVehiclePlugin() {
  
    left_steering_pid_ = common::PID(2000.0, 0.0, 300.0);
    right_steering_pid_ = common::PID(2000.0, 0.0, 300.0);
    left_steering_pid_.SetCmdMin(-5000.0);
    left_steering_pid_.SetCmdMax(5000.0);
    right_steering_pid_.SetCmdMin(-5000.0);
    right_steering_pid_.SetCmdMax(5000.0);

    rear_left_pid_ = common::PID(1000.0, 0.0, 1.0);
    rear_right_pid_ = common::PID(1000.0, 0.0, 1.0);
    rear_left_pid_.SetCmdMin(-5000.0);
    rear_left_pid_.SetCmdMax(5000.0);
    rear_right_pid_.SetCmdMin(-5000.0);
    rear_right_pid_.SetCmdMax(5000.0);

  
    last_update_time_ = common::Time(0);
    

    last_odom_pub_time_ = common::Time(0);


    target_linear_x_ = 0.0;
    target_angular_z_ = 0.0;
    last_update_time_ = common::Time(0);
}

StdMsgVehiclePlugin::~StdMsgVehiclePlugin() {
    update_connection_.reset();
    ros_node_.reset();
}

void StdMsgVehiclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    model_ = _model;

    if (!rclcpp::ok()) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("std_msg_vehicle_plugin"), "ROS 2未初始化");
        return;
    }

  
    ros_node_ = rclcpp::Node::make_shared("std_msg_vehicle_controller", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}}));
    RCLCPP_INFO(ros_node_->get_logger(), "标准消息车辆插件加载成功");

    fl_steer_joint_ = model_->GetJoint("front_left_steering_joint");
    fr_steer_joint_ = model_->GetJoint("front_right_steering_joint");
    rl_wheel_joint_ = model_->GetJoint("rear_left_wheel_joint");
    rr_wheel_joint_ = model_->GetJoint("rear_right_wheel_joint");

  
    wheelbase_ = _sdf->Get<double>("wheelbase", 3.0).first;       
    track_width_ = _sdf->Get<double>("track_width", 1.666).first; 
    wheel_radius_ = _sdf->Get<double>("wheel_radius", 0.3).first; 
    max_linear_x_ = _sdf->Get<double>("max_linear_x", 20.0).first; 
    max_angular_z_ = _sdf->Get<double>("max_angular_z", 1.0).first;
    max_steering_angle_ = _sdf->Get<double>("max_steering_angle", 0.7).first;

    if (!fl_steer_joint_ || !fr_steer_joint_ || !rl_wheel_joint_ || !rr_wheel_joint_) {
        RCLCPP_FATAL(ros_node_->get_logger(), "未找到车辆关节，请检查SDF关节名");
        return;
    }

    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&StdMsgVehiclePlugin::CmdVelCallback, this, std::placeholders::_1));


    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "odom", 10);

 
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&StdMsgVehiclePlugin::OnUpdate, this, std::placeholders::_1));
}


void StdMsgVehiclePlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    target_linear_x_ = std::clamp(msg->linear.x, -max_linear_x_, max_linear_x_);
    target_angular_z_ = std::clamp(msg->angular.z, -max_angular_z_, max_angular_z_);

    // 阿克曼转向：将角速度转换为前轮转向角
    // 公式: wz = vx * tan(steer) / wheelbase  =>  steer = atan(wz * wheelbase / vx)
    if (std::abs(target_linear_x_) > 0.01) {
        target_steering_angle_ = std::atan(target_angular_z_ * wheelbase_ / target_linear_x_);
        target_steering_angle_ = std::clamp(target_steering_angle_, -max_steering_angle_, max_steering_angle_);
    } else {
        // 线速度接近0时，阿克曼车辆无法转向
        target_steering_angle_ = 0.0;
    }
}


void StdMsgVehiclePlugin::OnUpdate(const common::UpdateInfo &_info) {

    rclcpp::spin_some(ros_node_);

    if (last_update_time_ == common::Time(0)) {
        last_update_time_ = _info.simTime;
        last_odom_pub_time_ = _info.simTime;  
        return;
    }


    double dt = (_info.simTime - last_update_time_).Double();
    last_update_time_ = _info.simTime;


    UpdateSteering(dt);
    UpdateSpeed(dt);


    double odom_time_diff = (_info.simTime - last_odom_pub_time_).Double();

    if (odom_time_diff >= odom_pub_interval_) {
        PublishOdom(_info.simTime);  
        last_odom_pub_time_ = _info.simTime; 
    }
}


void StdMsgVehiclePlugin::UpdateSteering(double dt) {

    double tan_steer = tan(target_steering_angle_);
    double target_fl_angle = std::atan2(wheelbase_ * tan_steer, wheelbase_ - 0.5 * track_width_ * tan_steer);
    double target_fr_angle = std::atan2(wheelbase_ * tan_steer, wheelbase_ + 0.5 * track_width_ * tan_steer);

    double current_fl_angle = fl_steer_joint_->Position(0);
    double current_fr_angle = fr_steer_joint_->Position(0);
    double fl_error = current_fl_angle - target_fl_angle;
    double fr_error = current_fr_angle - target_fr_angle;


    double fl_force = left_steering_pid_.Update(fl_error, dt);
    double fr_force = right_steering_pid_.Update(fr_error, dt);
    fl_steer_joint_->SetForce(0, fl_force);
    fr_steer_joint_->SetForce(0, fr_force);
}


void StdMsgVehiclePlugin::UpdateSpeed(double dt) {

    double target_angular_vel = target_linear_x_ / wheel_radius_;

    double current_rl_vel = rl_wheel_joint_->GetVelocity(0);
    double current_rr_vel = rr_wheel_joint_->GetVelocity(0);
    double rl_error = current_rl_vel - target_angular_vel;
    double rr_error = current_rr_vel - target_angular_vel;


    double rl_force = rear_left_pid_.Update(rl_error, dt);
    double rr_force = rear_right_pid_.Update(rr_error, dt);
    rl_wheel_joint_->SetForce(0, rl_force);
    rr_wheel_joint_->SetForce(0, rr_force);
}


void StdMsgVehiclePlugin::PublishOdom(const common::Time &sim_time) {
    auto odom_msg = nav_msgs::msg::Odometry();


    odom_msg.header.stamp = ros_node_->get_clock()->now();
    odom_msg.header.frame_id = "odom";      
    odom_msg.child_frame_id = "base_footprint";   


    auto base_link = model_->GetLink("base_footprint");
    auto world_pose = base_link->WorldPose();

    odom_msg.pose.pose.position.x = world_pose.Pos().X();
    odom_msg.pose.pose.position.y = world_pose.Pos().Y();
    odom_msg.pose.pose.position.z = world_pose.Pos().Z();

    odom_msg.pose.pose.orientation.x = world_pose.Rot().X();
    odom_msg.pose.pose.orientation.y = world_pose.Rot().Y();
    odom_msg.pose.pose.orientation.z = world_pose.Rot().Z();
    odom_msg.pose.pose.orientation.w = world_pose.Rot().W();

    odom_msg.pose.covariance.fill(0.0);


    auto world_linear_vel = base_link->WorldLinearVel();
    auto world_angular_vel = base_link->WorldAngularVel();

    odom_msg.twist.twist.linear.x = world_linear_vel.X();
    odom_msg.twist.twist.linear.y = 0.0;  
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = world_angular_vel.Z();

    odom_msg.twist.covariance.fill(0.0);

    odom_pub_->publish(odom_msg);
}


GZ_REGISTER_MODEL_PLUGIN(StdMsgVehiclePlugin)
}  