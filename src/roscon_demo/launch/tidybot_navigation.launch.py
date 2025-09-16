#!/usr/bin/env python3

"""
Launch file for TidyBot simulation with SLAM and Navigation.

This launch file starts:
1. TidyBot simulation environment
2. SLAM Toolbox for mapping
3. Nav2 navigation stack for autonomous navigation

Usage:
    ros2 launch roscon_demo tidybot_navigation.launch.py

Author: ROS2 Demo Package
License: Apache-2.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for TidyBot navigation setup."""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='true',
        description='Whether to run navigation'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'params',
            'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for Nav2 nodes'
    )
    
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    navigation = LaunchConfiguration('navigation')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    
    # TidyBot simulation launch
    tidybot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uol_tidybot'),
                'launch',
                'tidybot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # SLAM Toolbox launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(slam)
    )
    
    # Nav2 navigation launch - using individual components for better control
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
        condition=IfCondition(navigation)
    )
    
    # RViz launch with Nav2 configuration
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(rviz)
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_arg)
    ld.add_action(navigation_arg)
    ld.add_action(params_file_arg)
    ld.add_action(rviz_arg)
    
    # Add launch actions with timing
    ld.add_action(LogInfo(msg="Starting TidyBot simulation..."))
    ld.add_action(tidybot_launch)
    
    # Delay SLAM to ensure simulation is ready
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting SLAM Toolbox..."),
            slam_toolbox_launch
        ]
    ))
    
    # Delay Nav2 to ensure SLAM is ready
    ld.add_action(TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Starting Nav2 navigation stack..."),
            nav2_launch
        ]
    ))
    
    # Delay RViz to ensure everything else is ready
    ld.add_action(TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Starting RViz..."),
            rviz_launch
        ]
    ))
    
    return ld