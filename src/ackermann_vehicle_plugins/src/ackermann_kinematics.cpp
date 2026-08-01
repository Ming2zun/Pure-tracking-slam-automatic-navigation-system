// Copyright 2025 ackermann_vehicle_plugins contributors
// Licensed under the Apache License, Version 2.0

#include "ackermann_kinematics.h"
#include <algorithm>

namespace ackermann_vehicle_plugins {

// 可移植 PI 常量（不依赖 M_PI 宏）
static constexpr double PI = 3.14159265358979323846;

AckermannKinematics::AckermannKinematics(const AckermannParams& params)
    : params_(params) {}

void AckermannKinematics::setParams(const AckermannParams& params) {
    params_ = params;
}

double AckermannKinematics::cmdVelToSteer(double vx, double wz) const {
    // 钳位输入
    double vx_clamped = clamp(vx, -params_.max_speed, params_.max_speed);
    double wz_clamped = clamp(wz, -params_.max_angular_vel, params_.max_angular_vel);

    // 低速保护：阿克曼车辆停车时无法转向
    if (std::abs(vx_clamped) < params_.low_speed_threshold) {
        return 0.0;
    }

    // 阿克曼运动学逆解：δ = atan(wz * L / vx)
    double steer = std::atan(wz_clamped * params_.wheelbase / vx_clamped);
    return clamp(steer, -params_.max_steer_angle, params_.max_steer_angle);
}

SteerAngles AckermannKinematics::computeWheelAngles(double steer) const {
    SteerAngles angles;
    double tan_steer = std::tan(steer);

    // δ ≈ 0 时直接返回，避免 tan(0) 精度问题
    if (std::abs(tan_steer) < 1e-9) {
        angles.left  = 0.0;
        angles.right = 0.0;
        return angles;
    }

    double L = params_.wheelbase;
    double W = params_.track_width;

    // 阿克曼几何：左右前轮独立计算
    // fl = atan(L·tan(δ) / (L − 0.5·W·tan(δ)))
    // fr = atan(L·tan(δ) / (L + 0.5·W·tan(δ)))
    double numerator = L * tan_steer;
    angles.left  = std::atan2(numerator, L - 0.5 * W * tan_steer);
    angles.right = std::atan2(numerator, L + 0.5 * W * tan_steer);

    return angles;
}

double AckermannKinematics::linearToWheelOmega(double vx) const {
    if (params_.wheel_radius <= 0.0) {
        return 0.0;
    }
    double vx_clamped = clamp(vx, -params_.max_speed, params_.max_speed);
    return vx_clamped / params_.wheel_radius;
}

// ─── 静态工具函数 ───

double AckermannKinematics::clamp(double val, double lo, double hi) {
    return std::max(lo, std::min(val, hi));
}

double AckermannKinematics::radToDeg(double rad) {
    return rad * 180.0 / PI;
}

double AckermannKinematics::degToRad(double deg) {
    return deg * PI / 180.0;
}

}  // namespace ackermann_vehicle_plugins
