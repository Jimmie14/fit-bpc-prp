from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    launch_manhattan = LaunchConfiguration('launch_manhattan', default='true')

    project_share_dir = get_package_share_directory('project')
    ekf_config = os.path.join(project_share_dir, 'config', 'ekf.yaml')
    print(f"ekf conf: {ekf_config}")

    nodes = [
        DeclareLaunchArgument(
            'launch_manhattan',
            default_value='true',
            description='Launch the Manhattan node'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        Node(
            package='project',
            executable='manhattan',
            name='manhattan',
            output='screen',
            condition=IfCondition(launch_manhattan)
        )
    ]

    return LaunchDescription(nodes)