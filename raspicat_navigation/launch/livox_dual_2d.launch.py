import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription                                                  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    livox_dir=get_package_share_directory('livox_ros_driver2')
    livox_launch=os.path.join(livox_dir, 'launch_ROS2', 'msg_MID360_launch.py')

    livox_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch))

    p2l_param_path=os.path.join(
        get_package_share_directory('pointcloud_to_dual_laserscan'),
        'config', 'pointcloud_to_dual_laserscan.yaml'
    )

    p2l_node=Node(
        package='pointcloud_to_dual_laserscan',
        executable='pointcloud_to_dual_laserscan_node',
        name='pointcloud_to_dual_laserscan_node',
        parameters=[p2l_param_path],
        remappings=[('scan', 'scan/localization')],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(livox_ld)
    ld.add_action(p2l_node)
    return ld
