// Copyright 2025 ackermann_vehicle_plugins contributors
// Licensed under the Apache License, Version 2.0

#ifndef ACKERMANN_KINEMATICS_H
#define ACKERMANN_KINEMATICS_H

#include <cmath>

namespace ackermann_vehicle_plugins {

/// 阿克曼车辆几何参数（从 SDF/xacro 传入）
struct AckermannParams {
    double wheelbase       = 3.0;    ///< 前后轴距 L (m)
    double track_width     = 1.666;  ///< 左右轮距 W (m)
    double wheel_radius    = 0.3;    ///< 车轮半径 r (m)
    double max_steer_angle = 0.699;  ///< 最大转向角 (rad)
    double max_speed       = 20.0;   ///< 最大线速度 (m/s)
    double max_angular_vel = 1.0;    ///< 最大角速度 (rad/s)
    double low_speed_threshold = 0.01; ///< 低速阈值，低于此值时转向角置零 (m/s)
};

/// 左右前轮转向角
struct SteerAngles {
    double left;   ///< 左前轮转向角 (rad)
    double right;  ///< 右前轮转向角 (rad)
};

/// 阿克曼运动学纯计算类
///
/// 无 ROS / Gazebo 依赖，可独立编译和单元测试。
/// 职责：
///   1. cmd_vel (vx, wz) → 等效自行车转向角 δ
///   2. δ → 左右前轮转向角 (阿克曼几何)
///   3. vx → 后轮角速度 ω
class AckermannKinematics {
public:
    explicit AckermannKinematics(const AckermannParams& params);

    /// 更新参数（运行时可调）
    void setParams(const AckermannParams& params);

    /// 获取当前参数
    const AckermannParams& getParams() const { return params_; }

    // ────────────────────────────────────────────
    //  核心运动学计算
    // ────────────────────────────────────────────

    /// cmd_vel → 等效自行车转向角 δ
    ///
    /// 公式: δ = atan(wz * L / vx)
    /// 当 |vx| < low_speed_threshold 时返回 0（低速无法转向）
    /// 结果被钳位到 [-max_steer_angle, +max_steer_angle]
    double cmdVelToSteer(double vx, double wz) const;

    /// 等效转向角 δ → 左右前轮转向角（阿克曼几何）
    ///
    /// fl = atan(L·tan(δ) / (L − 0.5·W·tan(δ)))
    /// fr = atan(L·tan(δ) / (L + 0.5·W·tan(δ)))
    SteerAngles computeWheelAngles(double steer) const;

    /// 线速度 vx → 后轮角速度 ω (rad/s)
    ///
    /// 公式: ω = vx / r
    double linearToWheelOmega(double vx) const;

    // ────────────────────────────────────────────
    //  辅助工具函数
    // ────────────────────────────────────────────

    /// 钳位到 [lo, hi]
    static double clamp(double val, double lo, double hi);

    /// 弧度 → 角度
    static double radToDeg(double rad);

    /// 角度 → 弧度
    static double degToRad(double deg);

private:
    AckermannParams params_;
};

}  // namespace ackermann_vehicle_plugins

#endif  // ACKERMANN_KINEMATICS_H
