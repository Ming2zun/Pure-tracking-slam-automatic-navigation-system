# su7ultra_simulation

基于 **ROS 2 + Gazebo Classic** 的 **阿克曼转向车辆自主导航系统**，以小米 SU7 Ultra 为仿真物体。

集成 **Nav2 全栈导航**，同时提供通用阿克曼 Gazebo 插件包，支持显式运动学计算与参数化配置，适用于任意阿克曼车辆。

---

## 目录

- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [功能包总览](#功能包总览)
- [项目架构](#项目架构)
  - [1. su7ultra_description — 车辆模型描述](#1-su7ultra_description--车辆模型描述)
  - [2. su7ultra_navigation2 — Nav2 导航配置](#2-su7ultra_navigation2--nav2-导航配置)
  - [3. ackermann_vehicle_plugins — 通用阿克曼 Gazebo 插件](#3-ackermann_vehicle_plugins--通用阿克曼-gazebo-插件)
  - [4. ackermann_steering_angle — 转向角监测](#4-ackermann_steering_angle--转向角监测)
- [TF 坐标系树](#tf-坐标系树)
- [ROS 话题一览](#ros-话题一览)
- [常见问题排查](#常见问题排查)
- [完整导航工作流程](#完整导航工作流程)

---

## 环境要求

| 项目 | 版本 / 说明 |
|------|-------------|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble(只在这里进行了测试) |
| Gazebo | Classic 11.x（非 Ignition） |

**系统依赖安装：**

```bash
# ROS 2 基础依赖
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs \
                 ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-navigation2 \
                 ros-$ROS_DISTRO-nav2-bringup
```
应该还有很多包是漏掉的，大家自行安装。

---

## 快速开始

### 1. 编译

```bash
cd su7ultra_simulation
colcon build
source install/setup.bash
```

### 2. 安装 Gazebo 模型

```bash
cp -r src/su7ultra_description/models/* ~/.gazebo/models
```
如果没有`～/.gazebo`目录，请自行`mkdir`创建。

### 说明
1. 这个项目最初看上了su7ultra这个车的模型，然后自己也学习了鱼香ROS的ROS2教程，想着能不能以这个为基础，搭建基于阿克曼转向模型的仿真。
2. 感谢Ming2zun作者所作的工作，在此基础上，我将su7ultra车模由SDF按照鱼香ROS视频教程，把他分模块的写成urdf。
3. 在以上完成后，就能开始slam和导航了，然后解决了里面cpp代码存在的调用时间导致的tf时间问题，以适配鱼香ROS的slam_toolbox教程。
4. 完成SLAM后，在nav2部分，由于鱼香ROS的教程不是基于阿克曼转向模型的，并且由于nav2的参数量过大，且我在这方面并不熟悉，因此参数这边，还需要后续有人完善（我不清楚我能否调出来）。
5. 关于项目功能包，`four_wheeled_vehicle`包是最开始搭建的，里面存在的是非官方的阿克曼转向模型的代码，由于我想要阿克曼转向模型的代码不要写死在一个功能包中，因此把他抽出到`ackermann_vehicle_plugins`功能包中，然后`su7ultra_description`功能包里面，就在`gazebo_sim.launch.py`中添加了官方/非官方这两种方式的阿克曼转向模型的调用。然后由于我想要知阿克曼转向模型中，前轮的转向角度，故写了`ackermann_steering_angle`功能包。`nav_slam`这里我没有用到。

### 3. 启动 Gazebo 仿真

```bash
# 默认使用官方阿克曼插件
ros2 launch su7ultra_description gazebo_sim.launch.py

# 或切换为自研显式阿克曼插件
ros2 launch su7ultra_description gazebo_sim.launch.py drive_plugin:=explicit_ackermann
```

### 4. 启动 Nav2 导航

```bash
ros2 launch su7ultra_navigation2 navigation2.launch.py
```

在 RViz2 中：`2D Pose Estimate` 设定初始位姿 → `Nav2 Goal` 点击目标点。

---

## 功能包总览

| 功能包 | 构建类型 | 说明 |
|--------|----------|------|
| `su7ultra_description` | ament_cmake | SU7 Ultra URDF 模型、传感器、Gazebo 插件（支持双插件切换） |
| `su7ultra_navigation2` | ament_cmake | Nav2 导航栈完整配置（MPPI + SmacPlanner Hybrid-A*） |
| `ackermann_vehicle_plugins` | ament_cmake | 通用阿克曼 Gazebo 插件，参数化配置，适用于任意车型 |
| `ackermann_steering_angle` | ament_python | 通过 TF / JointState 实时监测前轮转向角 |

---

## 项目架构

### 1. su7ultra_description — 车辆模型描述

SU7 Ultra 阿克曼转向车辆的完整 URDF 模型定义，包含底盘、执行器、传感器及 Gazebo 仿真插件。

```
su7ultra_description/
├── urdf/vehicle/
│   ├── base.urdf.xacro              # 底盘（2000kg, 5.07×1.97×1.47m）
│   ├── vehicle.urdf.xacro           # 主入口，组装所有组件
│   ├── vehicle_simple.urdf.xacro    # 简化版（无传感器）
│   ├── actuator/
│   │   ├── steering.urdf.xacro      # 前轮转向关节（±0.7rad）
│   │   └── wheel.urdf.xacro         # 车轮（μ1=2.0, μ2=1.5）
│   ├── sensor/
│   │   ├── lidar3d.urdf.xacro       # 3D 激光雷达（32 线，270°）
│   │   ├── laser.urdf.xacro         # 2D 激光雷达（360°，Nav2 用）
│   │   ├── camera.urdf.xacro        # 前置摄像头（1280×720）
│   │   ├── imu.urdf.xacro           # IMU（200Hz）
│   │   └── gps.urdf.xacro           # GPS（10Hz）
│   └── plugin/
│       └── gazebo_control.xacro     # 双插件切换（官方 / 自研）
├── models/su7ultra/meshes/          # 3D 网格 + 纹理贴图
├── worlds/                          # Gazebo 世界文件
├── launch/
│   ├── gazebo_sim.launch.py         # Gazebo 启动（支持 drive_plugin 参数）
│   ├── display_robot.launch.py      # RViz 模型展示
│   └── rviz.launch.py               # 单独 RViz
├── CMakeLists.txt
└── package.xml
```

**车辆关键参数：**

| 参数 | 值 |
|------|-----|
| 整车质量 | 2000 kg |
| 车身尺寸 | 5.07 × 1.97 × 1.47 m |
| 车轮半径 / 宽度 | 0.30 m / 0.35 m |
| 前轮轮距 | 1.666 m |
| 前后轴距 | 2.990 m |
| 最大转向角 | ±0.7 rad（≈40°） |
| 最小转弯半径 | 4.39 m |

**传感器配置：**

| 传感器 | 位置 (x,y,z) m | 话题 | 更新率 |
|--------|----------------|------|--------|
| 3D 激光雷达 | (1.30, 0, 0.95) | `/points_raw` | 10Hz |
| 2D 激光雷达 | (1.30, 0, 0.30) | `/scan` | 5Hz |
| 前置摄像头 | (2.20, 0, 0.60) | `/image_raw` | 30Hz |
| IMU | (0, 0, 0.30) | `/imu_raw` | 200Hz |
| GPS | (-0.50, 0, 0.95) | `/gps/fix` | 10Hz |

**双驱动插件切换：**

`gazebo_control.xacro` 支持通过 `drive_plugin` 参数在两种插件间切换：

| 对比 | `ackermann_drive`（官方） | `explicit_ackermann`（自研） |
|------|--------------------------|---------------------------|
| 实现方式 | 物理引擎隐式约束 | 显式阿克曼公式计算 |
| 需要 `wheelbase` / `track_width` | 否 | 是 |
| 低速处理 | 物理引擎自然处理 | 阈值截止 |
| PID 可配 | 转向 5000/0/800, 速度 2000/0/2 | 转向 2000/0/300, 速度 1000/0/1 |
| 来源 | `gazebo_ros` 包 | `ackermann_vehicle_plugins` 包 |

```bash
# 官方插件（默认，推荐生产使用）
ros2 launch su7ultra_description gazebo_sim.launch.py

# 自研显式插件（学习/对比/实车控制场景）
ros2 launch su7ultra_description gazebo_sim.launch.py drive_plugin:=explicit_ackermann
```

---

### 2. su7ultra_navigation2 — Nav2 导航配置

基于 Nav2 导航栈的完整配置，针对阿克曼车辆特性进行了深度适配。

```
su7ultra_navigation2/
├── config/
│   ├── nav2_params.yaml             # Nav2 全栈参数
│   └── test_nav2_params.yaml        # 测试用参数
├── maps/
│   ├── test.yaml / test.pgm         # 测试地图（60×40m）
│   └── room.yaml / room.pgm         # 房间地图（36×34m）
├── behavior_trees/
│   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   └── navigate_through_poses_w_replanning_and_recovery.xml
├── launch/
│   └── navigation2.launch.py
├── CMakeLists.txt
└── package.xml
```
这里，nav2_params.yaml中，需要你把behavior_trees中的两个文件路径进行修改为你自己电脑上的路径。

**Nav2 核心模块配置：**

目前参数这块，配置的并没有很好，欢迎大家进行尝试。

| 模块 | 插件 | 关键参数 |
|------|------|----------|
| 定位 | AMCL | 粒子 500~5000，likelihood_field |
| 路径规划 | SmacPlannerHybrid | Reeds-Shepp，min_turning_r=4.39m |
| 路径跟踪 | MPPIController | Ackermann 运动学，batch_size=2000 |
| 局部代价地图 | VoxelLayer + InflationLayer | 15×15m 滑动窗口 |
| 全局代价地图 | StaticLayer + ObstacleLayer + InflationLayer | 静态地图 + 膨胀 |
| 行为恢复 | BackUp / DriveOnHeading / Wait | 无 Spin（阿克曼无法原地旋转） |

**阿克曼车辆适配要点：**

- **MPPI** — `motion_model: "Ackermann"`，8 个评价函数（ConstraintCritic w=4.0, ObstaclesCritic w=30.0, GoalCritic w=15.0 等）
- **SmacPlanner** — Reeds-Shepp 运动模型，支持前进/倒车，`reverse_penalty: 10.0`
- **行为树** — 移除 `Spin`，恢复策略：清除代价地图 → 前进 → 等待 → 后退
- **Footprint** — `[[2.535, 0.985], [2.535, -0.985], [-2.535, -0.985], [-2.535, 0.985]]`

---

### 3. ackermann_vehicle_plugins — 通用阿克曼 Gazebo 插件

通用 Gazebo 插件包，适用于**任意**阿克曼转向车辆。通过 SDF/xacro 参数化配置轴距、轮距、PID 等，零代码修改即可适配不同车型。

```
ackermann_vehicle_plugins/
├── include/
│   ├── ackermann_kinematics.h           # 纯数学运动学类（无 ROS/Gazebo 依赖）
│   └── explicit_ackermann_plugin.h      # Gazebo ModelPlugin
├── src/
│   ├── ackermann_kinematics.cpp         # 运动学实现
│   └── explicit_ackermann_plugin.cpp    # 插件实现（委托运动学类）
├── test/
│   └── test_ackermann_kinematics.cpp    # GTest 单元测试（16 个用例）
├── CMakeLists.txt
├── package.xml
└── README.md
```

**架构设计：**

运动学逻辑与 Gazebo 插件完全解耦。`AckermannKinematics` 是纯数学类，无 ROS/Gazebo 依赖，可独立编译和单元测试。

```
AckermannKinematics（纯数学类）
    ├── cmdVelToSteer(vx, wz)       →  δ = atan(wz·L / vx)
    ├── computeWheelAngles(δ)       →  fl, fr（阿克曼几何）
    └── linearToWheelOmega(vx)      →  ω = vx / r
                    │
                    ▼
ExplicitAckermannPlugin（Gazebo 插件）
    ├── 订阅 /cmd_vel → 调用运动学类计算目标转向角
    ├── PID 驱动转向关节到目标角度
    ├── PID 驱动后轮到目标角速度
    └── 发布 /odom 里程计
```

**SDF 可配置参数：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `wheelbase` | double | 3.0 | 前后轴距 L (m) |
| `track_width` | double | 1.666 | 左右轮距 W (m) |
| `wheel_radius` | double | 0.3 | 车轮半径 r (m) |
| `max_steer` | double | 0.699 | 最大转向角 (rad) |
| `max_speed` | double | 20.0 | 最大线速度 (m/s) |
| `max_angular_vel` | double | 1.0 | 最大角速度 (rad/s) |
| `steering_pid` | string | "2000 0 300" | 转向 PID (P I D) |
| `speed_pid` | string | "1000 0 1" | 速度 PID (P I D) |
| `pid_output_limit` | double | 5000 | PID 输出限幅 |
| `odom_frame` | string | "odom" | 里程计坐标系 |
| `base_frame` | string | "base_footprint" | 车体坐标系 |
| `odom_pub_rate` | double | 10 | 里程计发布频率 (Hz) |

**关节名约定（URDF 中必须一致）：**

```
front_left_steering_joint      ← 左前轮转向 (revolute)
front_right_steering_joint     ← 右前轮转向 (revolute)
rear_left_wheel_joint          ← 左后轮驱动 (continuous)
rear_right_wheel_joint         ← 右后轮驱动 (continuous)
```

**在自定义 URDF 中使用：**

```xml
<gazebo>
    <plugin name="explicit_ackermann_plugin"
            filename="libexplicit_ackermann_plugin.so">
        <wheelbase>2.990</wheelbase>
        <track_width>1.666</track_width>
        <wheel_radius>0.3</wheel_radius>
        <max_steer>0.699</max_steer>
        <max_speed>5.0</max_speed>
        <steering_pid>2000 0 300</steering_pid>
        <speed_pid>1000 0 1</speed_pid>
    </plugin>
</gazebo>
```

**vs. 官方 `gazebo_ros_ackermann_drive`：**

| 对比 | 官方插件（隐式） | 自研插件（显式） |
|------|-----------------|-----------------|
| 转向角计算 | 物理引擎隐式约束 | 数学公式显式计算 |
| 需要 wheelbase / track_width | 否 | 是 |
| 低速/停车转向 | 自然处理 | 阈值截止 |
| 轮胎滑移 | 物理引擎自然处理 | PID 与物理可能冲突 |
| 适用场景 | 生产仿真（推荐） | 教学、对比、实车控制 |

**单元测试：**

```bash
colcon test --packages-select ackermann_vehicle_plugins
colcon test-result --verbose
```

测试覆盖：直行/转向/倒车、阿克曼约束验证 `cot(δR) − cot(δL) = W/L`、低速阈值、速度钳位、多车型参数切换。

---

### 4. ackermann_steering_angle — 转向角监测

实时监测阿克曼车辆前轮实际转向角度的 ROS 2 节点，同时提供 TF 方式和 JointState 方式两种测量手段。

```
ackermann_steering_angle/
├── ackermann_steering_angle/
│   └── ackermann_steering_angle.py   # 转向角监测节点
├── launch/
│   └── steering_angle.launch.py      # 启动文件
├── package.xml
└── setup.py
```

**节点功能：**
- **TF 方式**：查询 `base_link → front_left/right_steering_link` 的变换，提取偏航角
- **JointState 方式**：订阅 `/joint_states`，读取转向关节位置

**发布话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vehicle/front_left_steering_angle` | Float32 | 左前轮转向角 (°) |
| `/vehicle/front_right_steering_angle` | Float32 | 右前轮转向角 (°) |
| `/vehicle/steering_angle` | Float32 | 左右平均转向角 (°) |

**启动：**

```bash
ros2 launch ackermann_steering_angle steering_angle.launch.py
```

---

## TF 坐标系树

```
map                              (Nav2: AMCL / 外部节点发布)
 └── odom                        (Gazebo 阿克曼插件发布)
      └── base_footprint         (阿克曼插件发布)
           └── base_link         (robot_state_publisher)
                ├── front_left_steering_link → front_left_wheel_link
                ├── front_right_steering_link → front_right_wheel_link
                ├── rear_left_wheel_link
                ├── rear_right_wheel_link
                ├── lidar3d_link
                ├── laser_link
                ├── camera_link
                ├── imu_link
                └── gps_link
```

---

## ROS 话题一览

### 传感器

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | LaserScan | 2D 激光雷达（360°，Nav2 用） |
| `/points_raw` | PointCloud2 | 3D 激光雷达（32 线） |
| `/image_raw` | Image | 前置摄像头 RGB |
| `/imu_raw` | Imu | IMU（200Hz） |
| `/gps/fix` | NavSatFix | GPS 定位 |

### 控制与导航

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | Twist | 速度指令 |
| `/odom` | Odometry | 里程计 |
| `/joint_states` | JointState | 6 个关节状态 |
| `/vehicle/steering_angle` | Float32 | 实时转向角监测 |

---

## 常见问题排查

| 问题 | 现象 | 解决 |
|------|------|------|
| TF 时间戳不匹配 | `timestamp earlier than transform cache` | 确保插件 `use_sim_time: true` |
| 关节 TF 缺失 | `No transform from steering_link to odom` | 检查 `joint_state_publisher` 插件是否加载 |
| Nav2 地图为空 | RViz 无地图显示 | 先 `2D Pose Estimate` 设定 AMCL 初始位姿 |
| mesh 加载失败 | 车辆显示白色 | `cp -r models/* ~/.gazebo/models` |
| 无法原地旋转 | Spin 行为无响应 | 正确行为，阿克曼车辆已用 DriveOnHeading 替代 |
| xacro 启动失败 | `unknown attribute(s): doc` | xacro 低版本不支持 `doc` 属性，已移除 |

---

## 完整建图工作流程

```bash
# 终端 1: Gazebo 仿真
ros2 launch su7ultra_description gazebo_sim.launch.py

# 终端 2: slam_toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# 终端 3: 控制小车移动
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 终端 4: rviz查看
ros2 launch four_wheeled_vehicle rviz.launch.py

# 终端 5: 保存地图文件为xxx
ros2 run nav2_map_server map_saver_cli -f xxx
```

## 完整导航工作流程

```bash
# 终端 1: Gazebo 仿真
ros2 launch su7ultra_description gazebo_sim.launch.py

# 终端 2: Nav2 导航
ros2 launch su7ultra_navigation2 navigation2.launch.py

# 终端 3（可选）: 转向角监测
ros2 launch ackermann_steering_angle steering_angle.launch.py

# RViz2: 2D Pose Estimate → Nav2 Goal
```

---

## 许可证

[Apache License 2.0](LICENSE)

**感谢：**
- [Ming2zun](https://github.com/Ming2zun)
- [喵了个水蓝蓝](https://www.bilibili.com/video/BV1kzEwzuEFw)