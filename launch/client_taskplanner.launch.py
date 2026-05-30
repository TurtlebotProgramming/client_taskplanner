from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('client_taskplanner'),
        'config', 'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='client_taskplanner',
            executable='turtlebot_fsm',
            name='turtlebot_fsm',
            output='screen',
            parameters=[config],
        ),
    ])
