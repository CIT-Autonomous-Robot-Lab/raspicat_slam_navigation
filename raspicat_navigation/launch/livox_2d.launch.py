import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription                                                  
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    livox_dir = get_package_share_directory('livox_ros_driver2')
    p2l_dir = get_package_share_directory('pointcloud_to_laserscan')
    livox_launch = os.path.join(livox_dir, 'launch_ROS2', 'msg_MID360_launch.py')
    p2l_launch = os.path.join(p2l_dir, 'launch', 'mid360_pointcloud2_to_laserscan.launch.py')

    livox_ld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch))

    p2l_ld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(p2l_launch))

    ld = LaunchDescription()
    ld.add_action(livox_ld)
    ld.add_action(p2l_ld)
    return ld

