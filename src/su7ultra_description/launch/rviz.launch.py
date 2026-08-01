import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('su7ultra_description')
    default_rviz_config_path = pkg_share + '/rviz/gazebo_sim.rviz'

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )

    return launch.LaunchDescription([
        rviz_node
    ])
