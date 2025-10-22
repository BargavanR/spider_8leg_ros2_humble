#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
*****************************************************************************************
*  Filename:       spider_gazebo.launch.py
*  Description:    spawn spider in gazebo and implement ros2_control
*  created by:    BARGAVAN R
*  Author:         SPIDER TEAM - MIT
*****************************************************************************************
'''

import os
from os.path import join

# ROS 2 launch system imports
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,          # Lets you declare parameters/arguments for the launch file
    IncludeLaunchDescription,       # To include other launch files
    AppendEnvironmentVariable,      # Adds paths to environment variables (like Gazebo resource path)
)
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory  # To locate package share directories
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ---------------------------
    # Get package directories
    # ---------------------------
    spider_warehouse = get_package_share_directory("spider_warehouse")  
    # Path to your warehouse package (contains worlds, models, etc.)

    # Declare world file location (default world is spider_warehouse.world)
    world_file = LaunchConfiguration(
        "world_file",
        default=join(spider_warehouse, "worlds", "spider_warehouse.world"),
    )
    
    # ---------------------------
    # Include Ignition Gazebo launch
    # ---------------------------
    gz_sim_share = get_package_share_directory("ros_gz_sim")  # Path to ros_gz_sim package
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")  # Include Gazebo’s launch file
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])  
            # Pass world file to Gazebo (run with -r = record)
        }.items(),
    )

    # ---------------------------
    # Robot description setup
    # ---------------------------
    pkg_share = get_package_share_directory("spider_description")  
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')  # Controller config
    urdf_file = os.path.join(pkg_share, "urdf", "spider_description.xacro")   # Path to robot xacro
    controllers_param = {'ros2_control': controllers_yaml}
    
    # ---------------------------
    # Rviz setup
    # ---------------------------

    rviz_share = get_package_share_directory("spider_rviz")
    rviz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(rviz_share, "launch", "launch.py")  # Include Gazebo’s launch file
            ),
        )

    # Expand xacro → URDF string
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),  # Runs xacro to convert into URDF
        value_type=str,
    )
    
    # ---------------------------
    # gz_bridge setup
    # ---------------------------
    
    bridge_params = os.path.join(
    pkg_share,
    'config',
    'bridge.yaml')

    start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '--ros-args',
        '-p',
        f'config_file:={bridge_params}',
    ],
    output='screen',
    )
   
    return LaunchDescription([
        # ---------------------------
        # Add Gazebo resource paths
        # ---------------------------
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(spider_warehouse, "worlds"),  # Allow Gazebo to find worlds
        ),
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(spider_warehouse, "models"),  # Allow Gazebo to find models
        ),

        # ---------------------------
        # Declare launch arguments
        # ---------------------------
        DeclareLaunchArgument("world_file", default_value=world_file),  # Let user override world

        # ---------------------------
        # Start Gazebo simulation
        # ---------------------------
        gz_sim,  # Start Gazebo with selected world

        # ---------------------------
        # Publish robot description to /robot_description
        # ---------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            
            output="screen",
            parameters=[{"robot_description": robot_description},
                        {"use_sim_time": True},
        ],  # Publishes TF + URDF
        ),

        # ---------------------------
        # Spawn robot entity in Gazebo
        # ---------------------------
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "spider_description",  # Robot name inside Gazebo
                "-topic", "robot_description",  # Spawn from /robot_description
                "-allow_renaming", "true",      # Avoid conflicts if duplicate name
                "-x", "-2.0499",                # X position
                "-y", "-5.969",                 # Y position
                "-z", "0.16",                   # Z position (lift off ground)
                "-R", "0.0",                    # Roll
                "-P", "0.0",                    # Pitch
                "-Y", "-1.57"                    # Yaw (90 degrees)
            ],
            output="screen",
        ),

        # ---------------------------
        # Start ros2_control manager
        # ---------------------------
        Node(
           package='controller_manager',
           executable='ros2_control_node',
           parameters=[controllers_param],  # Load controllers from YAML
           output='screen'
        ),

        # ---------------------------
        # Spawn joint_state_broadcaster
        # ---------------------------
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # ---------------------------
        # Spawn position_controller
        # ---------------------------
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # # ---------------------------
        # # Non-GUI joint_state_publisher
        # # ---------------------------
        # Node(
        #     package="joint_state_publisher",
        #     executable="joint_state_publisher",
        #     name="joint_state_publisher",
        #     output="screen",
        #     parameters=[controllers_param],  # Reads joint states for TF
        # ),
        # Node(
        # package="joint_state_publisher_gui",
        # executable="joint_state_publisher_gui",
        # name="joint_state_publisher_gui",
        # output="screen",
        # parameters=[{"robot_description": robot_description}],  # give robot model
        # ),
        # #     ---------------------------
        # # Optional GUI joint publisher
        # # ---------------------------
        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     name="joint_state_publisher_gui",
        #     output="screen",
        #     parameters=[controllers_param],
        # ),
        start_gazebo_ros_bridge_cmd,
        rviz_sim,
    ])
