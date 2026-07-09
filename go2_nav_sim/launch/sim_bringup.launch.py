# Copyright (c) 2026, Prosus Robotics
# SPDX-License-Identifier: Apache-2.0
"""Tier-0 Nav2 test bringup: real Go2 nav config + kinematic sim, no Gazebo.

Brings up map_server + AMCL + Nav2 + twist_mux against the *actual*
go2_robot_sdk params, and the kinematic sim node in place of the robot/LiDAR.
Runs in wall-clock time so nav2_params.yaml (use_sim_time: False) is unchanged.

    ros2 launch go2_nav_sim sim_bringup.launch.py
    ros2 launch go2_nav_sim sim_bringup.launch.py map:=/abs/office_map_v1.yaml \
        start_x:=1.0 start_y:=0.5 rviz:=true
    # stress localization / EKF:
    ros2 launch go2_nav_sim sim_bringup.launch.py odom_yaw_noise_std:=0.05
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('go2_nav_sim')
    sdk_share = get_package_share_directory('go2_robot_sdk')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(sim_share, 'worlds', 'example.yaml')
    default_params = os.path.join(sdk_share, 'config', 'nav2_params.yaml')
    default_twist_mux = os.path.join(sdk_share, 'config', 'twist_mux.yaml')
    default_rviz = os.path.join(sdk_share, 'config', 'single_robot_conf.rviz')

    map_arg = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    with_rviz = LaunchConfiguration('rviz')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')
    odom_xy_noise_std = LaunchConfiguration('odom_xy_noise_std')
    odom_yaw_noise_std = LaunchConfiguration('odom_yaw_noise_std')
    with_foxglove = LaunchConfiguration('foxglove')

    args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Occupancy map yaml to localize/plan on'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 params (defaults to the real go2_robot_sdk config)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Tier-0 runs in wall time; keep false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('odom_xy_noise_std', default_value='0.0',
                              description='Inject odom translational drift (m/sqrt(s))'),
        DeclareLaunchArgument('odom_yaw_noise_std', default_value='0.0',
                              description='Inject odom yaw drift (rad/sqrt(s))'),
        DeclareLaunchArgument('foxglove', default_value='false',
                              description='Run foxglove_bridge on :8765 for browser viz (headless/Docker)'),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_arg,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, default_twist_mux],
    )

    sim = Node(
        package='go2_nav_sim',
        executable='kinematic_sim',
        name='nav_sim',
        output='screen',
        parameters=[{
            'cmd_vel_topic': 'cmd_vel_out',
            'start_x': start_x,
            'start_y': start_y,
            'start_yaw': start_yaw,
            'odom_xy_noise_std': odom_xy_noise_std,
            'odom_yaw_noise_std': odom_yaw_noise_std,
        }],
    )

    # Turns Foxglove/RViz "Publish pose" clicks into Nav2 goals / waypoint routes.
    goal_relay = Node(
        package='go2_nav_sim',
        executable='goal_relay',
        name='goal_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='sim_rviz2',
        output='screen',
        condition=IfCondition(with_rviz),
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(with_foxglove),
        parameters=[{'port': 8765, 'address': '0.0.0.0'}],
    )

    return LaunchDescription(
        args + [localization, navigation, twist_mux, sim, goal_relay, rviz, foxglove])
