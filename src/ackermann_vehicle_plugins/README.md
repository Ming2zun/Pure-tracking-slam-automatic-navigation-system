# ackermann_vehicle_plugins

> Generic Gazebo plugins for **any** Ackermann steering vehicle.
> Configure wheelbase, track width, and PID via SDF/xacro — zero code changes needed.

## Features

| Feature | Description |
|---------|-------------|
| **Explicit Ackermann kinematics** | Computes left/right wheel angles from `δ = atan(wz·L/vx)` |
| **Parameter-driven** | All geometry from SDF — works with any 4-wheel Ackermann model |
| **Configurable PID** | Steering and speed PID via SDF `<steering_pid>` / `<speed_pid>` |
| **Dual backend** | Switch between official `gazebo_ros_ackermann_drive` and this plugin via xacro arg |
| **Standalone kinematics library** | `AckermannKinematics` class has no ROS/Gazebo dependency — unit-testable |

## Quick Start

### 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-select ackermann_vehicle_plugins
source install/setup.bash
```

### 2. Use in your URDF/xacro

```xml
<!-- In your gazebo_control.xacro -->
<gazebo>
    <plugin name="explicit_ackermann_plugin"
            filename="libexplicit_ackermann_plugin.so">
        <!-- Vehicle geometry (REQUIRED) -->
        <wheelbase>2.990</wheelbase>
        <track_width>1.666</track_width>
        <wheel_radius>0.3</wheel_radius>

        <!-- Limits -->
        <max_steer>0.699</max_steer>
        <max_speed>5.0</max_speed>
        <max_angular_vel>1.0</max_angular_vel>
        <low_speed_threshold>0.01</low_speed_threshold>

        <!-- PID: P I D -->
        <steering_pid>2000 0 300</steering_pid>
        <speed_pid>1000 0 1</speed_pid>
        <pid_output_limit>5000</pid_output_limit>

        <!-- Frames -->
        <odom_frame>odom</odom_frame>
        <base_frame>base_footprint</base_frame>
        <odom_pub_rate>10</odom_pub_rate>

        <!-- Topics -->
        <cmd_vel_topic>cmd_vel</cmd_vel_topic>
        <odom_topic>odom</odom_topic>
    </plugin>
</gazebo>
```

### 3. Switch plugins via launch argument

```bash
# Use official plugin (default)
ros2 launch my_robot gazebo_sim.launch.py

# Use explicit Ackermann plugin
ros2 launch my_robot gazebo_sim.launch.py drive_plugin:=explicit_ackermann
```

## SDF Parameters

### Kinematics

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheelbase` | double | 3.0 | Front-rear axle distance L (m) |
| `track_width` | double | 1.666 | Left-right wheel distance W (m) |
| `wheel_radius` | double | 0.3 | Wheel radius r (m) |
| `max_steer` | double | 0.699 | Maximum steering angle (rad) |
| `max_speed` | double | 20.0 | Maximum linear velocity (m/s) |
| `max_angular_vel` | double | 1.0 | Maximum angular velocity (rad/s) |
| `low_speed_threshold` | double | 0.01 | Below this vx, steering angle = 0 (m/s) |

### PID Control

| Parameter | Format | Default (Steer) | Default (Speed) | Description |
|-----------|--------|-----------------|-----------------|-------------|
| `steering_pid` | "P I D" | "2000 0 300" | — | Front wheel steering PID gains |
| `speed_pid` | "P I D" | — | "1000 0 1" | Rear wheel speed PID gains |
| `pid_output_limit` | double | 5000 | 5000 | PID output force limit |

### I/O

| Parameter | Default | Description |
|-----------|---------|-------------|
| `odom_frame` | "odom" | Odometry frame name |
| `base_frame` | "base_footprint" | Robot base frame name |
| `odom_pub_rate` | 10 | Odometry publish rate (Hz) |
| `cmd_vel_topic` | "cmd_vel" | Velocity command subscription topic |
| `odom_topic` | "odom" | Odometry publish topic |

## Joint Name Convention

Your URDF/SDF **must** define these exact joint names:

```
front_left_steering_joint     ← Left front wheel steering (revolute)
front_right_steering_joint    ← Right front wheel steering (revolute)
rear_left_wheel_joint         ← Left rear drive wheel (continuous)
rear_right_wheel_joint        ← Right rear drive wheel (continuous)
```

## Architecture

```
ackermann_vehicle_plugins/
├── include/
│   ├── ackermann_kinematics.h        # Pure math class (no ROS/Gazebo)
│   └── explicit_ackermann_plugin.h   # Gazebo ModelPlugin
├── src/
│   ├── ackermann_kinematics.cpp
│   └── explicit_ackermann_plugin.cpp
├── test/
│   └── test_ackermann_kinematics.cpp # GTest unit tests
├── CMakeLists.txt
├── package.xml
└── README.md
```

### Kinematics Class

```
cmd_vel (vx, wz)
    │
    ▼
cmdVelToSteer(vx, wz)          →  δ = atan(wz·L / vx)
    │
    ▼
computeWheelAngles(δ)          →  fl = atan(L·tan(δ) / (L − 0.5·W·tan(δ)))
                                   fr = atan(L·tan(δ) / (L + 0.5·W·tan(δ)))
    │
    ▼
linearToWheelOmega(vx)         →  ω = vx / r
```

### vs. Official Plugin

| Aspect | `gazebo_ros_ackermann_drive` | `explicit_ackermann_plugin` |
|--------|------|------|
| Steering angle | Implicit (physics engine) | Explicit (math formula) |
| Needs `wheelbase` param | No | Yes |
| Needs `track_width` param | No | Yes |
| Low-speed handling | Natural | Threshold cutoff |
| Tire scrub | Physics-natural | PID vs physics conflict possible |
| Best for | Production simulation | Education, comparison, real-car control |

## Unit Tests

```bash
colcon test --packages-select ackermann_vehicle_plugins
colcon test-result --verbose
```

Tests cover:
- Straight-line / turning / reversing steering
- Ackermann geometric constraint (`cot(δ_R) - cot(δ_L) = W/L`)
- Low-speed threshold behavior
- Speed clamping
- Runtime parameter switching
- Different vehicle configurations (sedan vs kart)

## License

Apache-2.0
