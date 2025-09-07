#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*        =============================================
*                 SPIDER - 8 LEGGED
*        =============================================
*
*
*  Filename:			spawn_spider_launch.py
*  Description:         Use this file to spawn spider inside spider warehouse world in the gazebo simulator and publish robot states.
*  Created:				07/09/2025
*  Last Modified:	    07/09/2025
*  Modified by:         BARGAVAN R
*  Author:				SPIDER - MIT Team
*  
*****************************************************************************************
'''
import launch
import launch_ros
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='spider_description').find('spider_description')

    xacro_file_spider = os.path.join(pkg_share, 'models/','spider/', 'spider_description.xacro')
    assert os.path.exists(xacro_file_spider), "The box_bot.xacro doesnt exist in "+str(xacro_file_spider)
    robot_description_config_spider = xacro.process_file(xacro_file_spider)
    robot_description_spider = robot_description_config_spider.toxml()

    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('spider_description'), 'launch', 'start_world_launch.py'),
        )
    )
    robot_state_publisher_node_spider = launch_ros.actions.Node(
        package='robot_state_publisher',
        name='spider_RD',
        parameters=[{"robot_description":robot_description_spider}],
        remappings=[{'robot_description', 'robot_description_spider'}]
    )


    spawn_spider = launch_ros.actions.Node(
    	package='gazebo_ros', 
        name='spider_spawner',
    	executable='spawn_entity.py',
        # arguments=['-entity', 'ebot', '-topic', 'robot_description_ebot', '-x', '1.1', '-y', '4.35', '-z', '0.1', '-Y', '3.14'],
        # arguments=['-entity', 'ebot', '-topic', 'robot_description_ebot', '-x', '0.0', '-y', '0.0', '-z', '0.1', '-Y', '0.0'],
        arguments=['-entity', 'spider', '-topic', 'robot_description_spider', '-x', '1.84', '-y', '-9.05', '-z', '0.1', '-Y', '3.14'],
        output='screen'
    )    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        robot_state_publisher_node_spider,
        spawn_spider
        #static_transform
    ])