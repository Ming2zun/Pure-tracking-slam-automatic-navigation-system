// Copyright 2025 ackermann_vehicle_plugins contributors
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include "ackermann_kinematics.h"

using namespace ackermann_vehicle_plugins;

// 可移植 PI 常量
static constexpr double PI = 3.14159265358979323846;

// ─── 测试夹具 ───
class AckermannKinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 使用 SU7 Ultra 参数
        params_.wheelbase = 2.990;
        params_.track_width = 1.666;
        params_.wheel_radius = 0.3;
        params_.max_steer_angle = 0.699;
        params_.max_speed = 20.0;
        params_.max_angular_vel = 1.0;
        params_.low_speed_threshold = 0.01;
        kinematics_ = std::make_unique<AckermannKinematics>(params_);
    }

    AckermannParams params_;
    std::unique_ptr<AckermannKinematics> kinematics_;
};

// ═══════════════════════════════════════════════
//  cmdVelToSteer 测试
// ═══════════════════════════════════════════════

TEST_F(AckermannKinematicsTest, CmdVelToSteer_StraightLine) {
    // 直行：wz=0 → steer=0
    EXPECT_DOUBLE_EQ(kinematics_->cmdVelToSteer(1.0, 0.0), 0.0);
}

TEST_F(AckermannKinematicsTest, CmdVelToSteer_PureTurn) {
    // vx=1.0, wz=0.5 → steer = atan(0.5 * 2.99 / 1.0) ≈ 0.982 rad
    // 但被钳位到 max_steer_angle=0.699
    double steer = kinematics_->cmdVelToSteer(1.0, 0.5);
    EXPECT_NEAR(steer, 0.699, 1e-3);
}

TEST_F(AckermannKinematicsTest, CmdVelToSteer_SmallAngle) {
    // vx=5.0, wz=0.1 → steer = atan(0.1 * 2.99 / 5.0) ≈ 0.0598
    double steer = kinematics_->cmdVelToSteer(5.0, 0.1);
    EXPECT_NEAR(steer, std::atan(0.1 * 2.99 / 5.0), 1e-6);
}

TEST_F(AckermannKinematicsTest, CmdVelToSteer_LowSpeed) {
    // |vx| < threshold → steer=0
    EXPECT_DOUBLE_EQ(kinematics_->cmdVelToSteer(0.0, 0.5), 0.0);
    EXPECT_DOUBLE_EQ(kinematics_->cmdVelToSteer(0.005, 0.5), 0.0);
}

TEST_F(AckermannKinematicsTest, CmdVelToSteer_NegativeSpeed) {
    // 倒车时转向角反向
    double steer_fwd = kinematics_->cmdVelToSteer(2.0, 0.3);
    double steer_rev = kinematics_->cmdVelToSteer(-2.0, 0.3);
    EXPECT_NEAR(steer_fwd, -steer_rev, 1e-9);
}

TEST_F(AckermannKinematicsTest, CmdVelToSteer_Symmetry) {
    // 左右转向对称
    double steer_left  = kinematics_->cmdVelToSteer(3.0, 0.2);
    double steer_right = kinematics_->cmdVelToSteer(3.0, -0.2);
    EXPECT_NEAR(steer_left, -steer_right, 1e-9);
}

// ═══════════════════════════════════════════════
//  computeWheelAngles 测试
// ═══════════════════════════════════════════════

TEST_F(AckermannKinematicsTest, WheelAngles_ZeroSteer) {
    // steer=0 → 两个轮子都是 0
    auto angles = kinematics_->computeWheelAngles(0.0);
    EXPECT_NEAR(angles.left, 0.0, 1e-9);
    EXPECT_NEAR(angles.right, 0.0, 1e-9);
}

TEST_F(AckermannKinematicsTest, WheelAngles_LeftTurn) {
    // 左转（steer > 0）→ 左轮角度 > 右轮角度
    auto angles = kinematics_->computeWheelAngles(0.3);
    EXPECT_GT(angles.left, angles.right);
    EXPECT_GT(angles.left, 0.0);
    EXPECT_GT(angles.right, 0.0);
}

TEST_F(AckermannKinematicsTest, WheelAngles_RightTurn) {
    // 右转（steer < 0）→ 左轮角度 < 右轮角度（都是负值）
    auto angles = kinematics_->computeWheelAngles(-0.3);
    EXPECT_LT(angles.left, 0.0);
    EXPECT_LT(angles.right, 0.0);
    // 右轮的绝对值更大
    EXPECT_LT(angles.left, angles.right);
}

TEST_F(AckermannKinematicsTest, WheelAngles_AckermannConstraint) {
    // 阿克曼约束验证：
    // cot(δ_right) - cot(δ_left) = W / L
    double steer = 0.4;
    auto angles = kinematics_->computeWheelAngles(steer);

    double cot_left  = 1.0 / std::tan(angles.left);
    double cot_right = 1.0 / std::tan(angles.right);

    EXPECT_NEAR(cot_right - cot_left,
                params_.track_width / params_.wheelbase, 1e-6);
}

// ═══════════════════════════════════════════════
//  linearToWheelOmega 测试
// ═══════════════════════════════════════════════

TEST_F(AckermannKinematicsTest, LinearToWheelOmega_Basic) {
    // ω = vx / r
    double omega = kinematics_->linearToWheelOmega(3.0);
    EXPECT_NEAR(omega, 3.0 / 0.3, 1e-6);
}

TEST_F(AckermannKinematicsTest, LinearToWheelOmega_Zero) {
    EXPECT_DOUBLE_EQ(kinematics_->linearToWheelOmega(0.0), 0.0);
}

TEST_F(AckermannKinematicsTest, LinearToWheelOmega_Clamped) {
    // max_speed=20 → 即使传入 30，也只输出 20/0.3
    double omega = kinematics_->linearToWheelOmega(30.0);
    EXPECT_NEAR(omega, 20.0 / 0.3, 1e-6);
}

// ═══════════════════════════════════════════════
//  辅助函数测试
// ═══════════════════════════════════════════════

TEST_F(AckermannKinematicsTest, Clamp) {
    EXPECT_DOUBLE_EQ(AckermannKinematics::clamp(5.0, 0.0, 10.0), 5.0);
    EXPECT_DOUBLE_EQ(AckermannKinematics::clamp(-1.0, 0.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ(AckermannKinematics::clamp(15.0, 0.0, 10.0), 10.0);
}

TEST_F(AckermannKinematicsTest, RadDegConversion) {
    EXPECT_NEAR(AckermannKinematics::radToDeg(PI), 180.0, 1e-9);
    EXPECT_NEAR(AckermannKinematics::degToRad(180.0), PI, 1e-9);
    EXPECT_NEAR(AckermannKinematics::radToDeg(
        AckermannKinematics::degToRad(45.0)), 45.0, 1e-9);
}

// ═══════════════════════════════════════════════
//  参数切换测试（验证通用性）
// ═══════════════════════════════════════════════

TEST_F(AckermannKinematicsTest, DifferentVehicleParams) {
    // 换一辆小型卡丁车
    AckermannParams kart;
    kart.wheelbase = 1.2;
    kart.track_width = 0.8;
    kart.wheel_radius = 0.15;
    kart.max_steer_angle = 0.8;
    kart.max_speed = 10.0;
    kart.max_angular_vel = 2.0;
    kart.low_speed_threshold = 0.01;

    AckermannKinematics kart_kin(kart);

    // 同样的 cmd_vel，不同轴距 → 不同转向角
    double steer_su7  = kinematics_->cmdVelToSteer(2.0, 0.3);
    double steer_kart = kart_kin.cmdVelToSteer(2.0, 0.3);
    EXPECT_NE(steer_su7, steer_kart);

    // 卡丁车轴距更短 → 同样的 wz 需要更大的转向角
    EXPECT_GT(std::abs(steer_kart), std::abs(steer_su7));
}

TEST_F(AckermannKinematicsTest, SetParamsRuntime) {
    // 运行时修改参数
    AckermannParams new_params = params_;
    new_params.wheelbase = 1.5;
    kinematics_->setParams(new_params);

    EXPECT_DOUBLE_EQ(kinematics_->getParams().wheelbase, 1.5);

    // 验证参数生效
    double steer = kinematics_->cmdVelToSteer(3.0, 0.2);
    EXPECT_NEAR(steer, std::atan(0.2 * 1.5 / 3.0), 1e-6);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
