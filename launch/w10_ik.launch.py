from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # W10 Inverse Kinematics Node
    w10_ik_node = Node(
        package='w10_kinematics',
        executable='w10_ik_node',
        name='w10_ik_node',
        output='screen',
    )

    ld.add_action(w10_ik_node)

    return ld
