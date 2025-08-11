from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('shahed'),
        'urdf',
        'shahed.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='shahed',
            executable='imu_tf_broadcaster.py',
            name='imu_tf_broadcaster'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('shahed'), 'urdf', 'shahed.rviz')]
        )
    ])
