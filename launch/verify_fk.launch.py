import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('twoplanar_arm')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'twoplanar_arm.urdf')

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
            executable='forward_kinematics_verifier',
            name='forward_kinematics_verifier',
            output='screen'
        )
    ])
