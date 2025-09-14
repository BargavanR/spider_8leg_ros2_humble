#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
*****************************************************************************************
*  Filename:       warehouse.launch.py
*  Description:    Launch Ignition Gazebo Fortress world with a custom warehouse
*  Modified by:    BARGAVAN R
*  Author:         SPIDER TEAM - MIT
*****************************************************************************************
'''

import os
from os.path import join

# Core ROS 2 launch imports
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,          # Lets us declare launch arguments
    IncludeLaunchDescription,       # Allows including another launch file
    AppendEnvironmentVariable,      # Used to add paths to environment variables
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory  # Finds package share dirs


def generate_launch_description():
    # ------------------------------------------------
    # Locate the share directory of the warehouse pkg
    # ------------------------------------------------
    # spider_warehouse package contains worlds/ and models/
    spider_warehouse = get_package_share_directory("spider_warehouse")

    # ------------------------------------------------
    # World file configuration
    # ------------------------------------------------
    # Create a launch argument for the world file.
    # Default is spider_warehouse.world inside the worlds/ folder.
    world_file = LaunchConfiguration(
        "world_file",
        default=join(spider_warehouse, "worlds", "spider_warehouse.world")
    )

    # ------------------------------------------------
    # Setup Gazebo (ros_gz_sim) bridge
    # ------------------------------------------------
    # Get share directory of ros_gz_sim (Ignition ↔ ROS bridge)
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    # Include the default gz_sim.launch.py
    # Pass gz_args → world file path, with "-r" (run immediately).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    # ------------------------------------------------
    # Return launch description with actions
    # ------------------------------------------------
    return LaunchDescription([
        # Add warehouse/worlds folder to Gazebo’s resource search path
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(spider_warehouse, "worlds")
        ),

        # Add warehouse/models folder to Gazebo’s resource search path
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(spider_warehouse, "models")
        ),

        # Declare world_file argument so user can override it in CLI
        DeclareLaunchArgument("world_file", default_value=world_file),

        # Finally, launch Gazebo with the chosen world
        gz_sim,
    ])
