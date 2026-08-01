import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name_in_model = "four_wheeled_vehicle"
    urdf_tutorial_path = get_package_share_directory('four_wheeled_vehicle')
    default_model_path = urdf_tutorial_path + '/urdf/vehicle/vehicle.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/worlds/ackermann_test.world'
    default_rviz_config_path = urdf_tutorial_path + '/rviz/gazebo_sim.rviz'
    model_path = os.path.join(urdf_tutorial_path, 'models')
    plugin_path = os.path.join(urdf_tutorial_path, '../..', 'lib', 'four_wheeled_vehicle')

    os.environ['GAZEBO_MODEL_PATH'] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{model_path}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{os.environ.get('GAZEBO_PLUGIN_PATH', '')}:{plugin_path}"

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
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
        ])

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, '-x', '0', '-y', '0', '-z', '0.325'])  # 0.324+0.001，虽然写0.5也是一样的，gazebo初始化的时候，物体会下沉到地面上。
    
    odom_baselink_tf_node = launch_ros.actions.Node(
        package='four_wheeled_vehicle',
        executable='odom_baselinkTF',
        name='odom_baselink_tf',
        output='screen',
        parameters=[{"use_sim_time": True}]
    )

    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        odom_baselink_tf_node,
        launch_gazebo,
        spawn_entity_node,
        # rviz_node
    ])
