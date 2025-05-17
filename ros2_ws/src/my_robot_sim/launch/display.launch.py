from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'my_robot_sim',
        'urdf',
        'my_robot.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                os.getenv('HOME'),
                'ros2_ws',
                'src',
                'my_robot_sim',
                'rviz',
                'robot_config.rviz')],
            output='screen'
        )
    ])

