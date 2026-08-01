# // # Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
# // #                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
# // #
# // # Licensed under the Apache License, Version 2.0 (the "License");
# // # you may not use this file except in compliance with the License.
# // # You may obtain a copy of the License at
# // #
# // #     http://www.apache.org/licenses/LICENSE-2.0
# // #
# // # Unless required by applicable law or agreed to in writing, software
# // # distributed under the License is distributed on an "AS IS" BASIS,
# // # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# // # See the License for the specific language governing permissions and
# // # limitations under the License.












import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 用法：
#   ros2 launch four_wheeled_vehicle vehicle_gazebo_ok.launch.py

def generate_launch_description():
    declare_use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='是否显示关节控制GUI（未启用 joint_state_publisher_gui）'
    )
    use_gui = LaunchConfiguration('use_gui')

    # 路径
    pkg_path = FindPackageShare('four_wheeled_vehicle').find('four_wheeled_vehicle')
    model_path = os.path.join(pkg_path, 'models')
    model_sdf  = os.path.join(model_path, 'four_wheeled_vehicle', 'model_sensor.sdf')
    world_file = os.path.join(pkg_path, 'worlds', '2d.world')
    plugin_path = os.path.join(pkg_path, '../..', 'lib', 'four_wheeled_vehicle')

    # 环境变量
    os.environ['GAZEBO_MODEL_PATH']  = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{model_path}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{os.environ.get('GAZEBO_PLUGIN_PATH', '')}:{plugin_path}"

    print("plugin_path:", plugin_path)
    print("world_file:", world_file)
    print("model_path:", model_path)
    print("model_sdf :", model_sdf)

    # 启动 Gazebo（务必带 ros_init 与 ros_factory）
    start_gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # 载入 SDF 模型
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'four_wheeled_car',
            '-file', model_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '1.0'
        ],
        output='screen'
    )









    # 发布TF
    stf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_lidar3d',
        arguments=['0', '0', '2.25', '0', '0', '0', '0', 'base_link', 'lidar3d_link'],
        output='screen'
    )
    stf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_gps',
        arguments=['0', '0', '2.25', '0', '0', '0', '0', 'base_link', 'gps_link'],
        output='screen'
    )
    stf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_imu',
        arguments=['0', '0', '2.25', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    stf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_camera',
        arguments=['0', '0', '2.35', '0', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    stf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom',
        arguments=['0', '0', '2', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    odom_map_tf_node = Node(
        package='four_wheeled_vehicle',
        executable='odom_mapTF',
        name='odom_map_tf_node',
        output='screen'
)

    return LaunchDescription([
        declare_use_gui_arg,
        start_gazebo,
        spawn_entity,
        stf_lidar,   
        stf_gps,
        stf_imu,
        stf_camera,
        stf_base,
        odom_map_tf_node
    ])
