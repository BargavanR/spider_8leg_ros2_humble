#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:       warehouse.launch.py
*  Description:    Launch Ignition Gazebo Fortress world 
*  Modified by:    BARGAVAN R
*  Author:         SPIDER TEAM - MIT 
*****************************************************************************************
'''
import os
from os.path import join
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    spider_warehouse = get_package_share_directory("spider_warehouse")

    world_file = LaunchConfiguration(
        "world_file",
        default= join(spider_warehouse,"worlds","spider_warehouse.world")
    )
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share,"launch","gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file," -r'"])
        }.items(),
    )

    return LaunchDescription([
    # Set resource paths for Gazebo
    AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=join(spider_warehouse, "worlds")
    ),
    AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=join(spider_warehouse, "models")
    ),
    # Declare launch arguments
    DeclareLaunchArgument("world_file", default_value=world_file),
    
    # Launch Gazebo
    gz_sim,
])
