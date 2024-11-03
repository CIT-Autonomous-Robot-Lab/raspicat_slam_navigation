import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription                                                  

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('raspicat_navigation') 
    template_launch_path = os.path.join(pkg_dir, 'launch', 'template.launch.py')
    config_dir = os.path.join(pkg_dir, 'config')
    param_dir = os.path.join(config_dir, 'param')
    map_dir = os.path.join(config_dir, 'map', 'tsudanuma_campus')

    nav2_params_path = os.path.join(
            param_dir, 'nav2.param.yaml')
    gnss2map_params_path = os.path.join(
            param_dir, 'tsudanuma_gnss2map.param.yaml')
    loc_map_path = os.path.join(
            map_dir, 'localization', 'map_tsudanuma_campus.yaml')
    nav_map_path = os.path.join(
            map_dir, 'navigation', 'map_tsudanuma_campus.yaml')

    use_sim_time = 'False'
    use_rviz = 'true'
    use_wall_tracking = 'false'
    use_gnss = 'true'

    launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(template_launch_path), 
            launch_arguments={
                'use_sim_time': use_sim_time, 
                'use_rviz': use_rviz, 
                'loc_map': loc_map_path, 
                'nav_map': nav_map_path, 
                'nav2_params_file': nav2_params_path, 
                'gnss2map_params_file': gnss2map_params_path, 
                'use_wall_tracking': use_wall_tracking, 
                'use_gnss': use_gnss}.items()
            )
    ld = LaunchDescription()
    ld.add_action(launch)
    return ld

