# Copyright 2023 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    loc_map_yaml_file = LaunchConfiguration('loc_map')
    nav_map_yaml_file = LaunchConfiguration('nav_map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    gnss2map_params_file = LaunchConfiguration('gnss2map_params_file')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_wall_tracking = LaunchConfiguration('use_wall_tracking')
    use_gnss = LaunchConfiguration('use_gnss')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')
    declare_arg_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Set "true" to launch rviz.')
    declare_loc_map_yaml = DeclareLaunchArgument(
        'loc_map', default_value='', 
        description='Full path to map yaml file for localization to load')
    declare_nav_map_yaml = DeclareLaunchArgument(
        'nav_map', default_value='',
        description='Full path to map yaml file for navigation to load')
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file', default_value='',
        description='Full path to the nav2, emcl2 and wall_tracking parameters file to use for all launched nodes')
    declare_gnss2map_params_file = DeclareLaunchArgument(
        'gnss2map_params_file', default_value='',
        description='Full path to the parameters in gnss2map package to use for all launched nodes')
    declare_container_name = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    declare_use_wall_tracking = DeclareLaunchArgument(
        'use_wall_tracking', default_value='false', 
        description='Whether to use wall_tracking')
    declare_use_gnss = DeclareLaunchArgument(
        'use_gnss', default_value='true', 
        description='Whether to use gnss')

    container_name_full = (namespace, '/', container_name)

    param_substitutions = {'use_sim_time': use_sim_time}
    
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    lifecycle_nodes = [
        'loc_map_server',
        'nav_map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                condition=IfCondition(use_gnss),
                package="gnss2map", 
                name="gauss_kruger_node", 
                executable="gauss_kruger_node", 
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[gnss2map_params_file],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                condition=IfCondition(use_wall_tracking), 
                package="wall_tracking_executor", 
                executable="wall_tracking_node", 
                name="wall_tracking_node", 
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='loc_map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"yaml_filename": loc_map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings+[('map', '/map/localization')]),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='nav_map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"yaml_filename": nav_map_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings+[('map', '/map')]),
            Node(
                package='emcl2',
                executable='emcl2_node',
                name='emcl2',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_gnss_reset': use_gnss,
                               'use_wall_tracking': use_wall_tracking}],
                # arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings+[('map', '/map/localization')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings+[('/goal_pose', 'goal_pose/fake')]),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='loc_map_server',
                parameters=[configured_params],
                remappings=remappings+[('map', '/map/localization')]),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='nav_map_server',
                parameters=[configured_params],
                remappings=remappings+[('map', '/map')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    rviz_config_file = os.path.join(get_package_share_directory('raspicat_navigation'), 
                                    'config', 'rviz', 'nav2.rviz')
    rviz2 = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        ros_arguments=['--log-level', 'WARN'],
        condition=IfCondition(use_rviz))

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_loc_map_yaml)
    ld.add_action(declare_nav_map_yaml)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav2_params_file)
    ld.add_action(declare_gnss2map_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_container_name)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_log_level)
    ld.add_action(declare_arg_use_rviz)
    ld.add_action(declare_use_wall_tracking)
    ld.add_action(declare_use_gnss)

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    
    ld.add_action(rviz2)
    
    return ld
  
