from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    ekf_config = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share/project/config/ekf.yaml'
    )

    return LaunchDescription([

        # Your C++ node
        Node(
            package='project',
            executable='manhattan',
            name='manhattan',
            output='screen'
        ),

        # EKF from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])