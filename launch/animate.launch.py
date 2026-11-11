import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('twoplanar_arm')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'twoplanar_arm.urdf')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'twoplanar_arm.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='twoplanar_arm',
            executable='sine_wave_joint_publisher',
            name='sine_wave_joint_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
