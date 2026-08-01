import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name_in_model = "su7ultra"
    urdf_tutorial_path = get_package_share_directory('su7ultra_description')
    default_model_path = urdf_tutorial_path + '/urdf/vehicle.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/worlds/ackermann_test.world'
    default_rviz_config_path = urdf_tutorial_path + '/rviz/gazebo_sim.rviz'
    model_path = os.path.join(urdf_tutorial_path, 'models')

    os.environ['GAZEBO_MODEL_PATH'] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{model_path}"

    # ── Launch 参数 ──
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF/xacro 文件路径')

    action_declare_drive_plugin = launch.actions.DeclareLaunchArgument(
        name='drive_plugin', default_value='explicit_ackermann',
        description='驱动插件: ackermann_drive(官方) | explicit_ackermann(自研)',
        choices=['ackermann_drive', 'explicit_ackermann'])

    # ── 将 drive_plugin 参数传递给 xacro ──
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command([
            'xacro ', launch.substitutions.LaunchConfiguration('model'),
            ' drive_plugin:=', launch.substitutions.LaunchConfiguration('drive_plugin'),
        ]),
        value_type=str)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
      	launch_arguments=[('world', default_world_path), ('verbose', 'true'),
            ('gui_required', 'true')  # 添加这个实现关闭gazebo客户端同时自动关闭服务端
        ]
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, '-x', '0', '-y', '0', '-z', '0.325'])  # 0.324+0.001，虽然写0.5也是一样的，gazebo初始化的时候，物体会下沉到地面上。

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_declare_drive_plugin,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
    ])
