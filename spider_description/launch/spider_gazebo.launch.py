import os
from os.path import join

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get paths
    spider_warehouse = get_package_share_directory("spider_warehouse")
    world_file = LaunchConfiguration(
        "world_file",
        default=join(spider_warehouse, "worlds", "spider_warehouse.world"),
    )
    
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    pkg_share = get_package_share_directory("spider_description")
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    urdf_file = os.path.join(pkg_share, "urdf", "spider_description.xacro")
    controllers_param = {'ros2_control': controllers_yaml}

    # Expand the xacro and make sure it's a string
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str,
    )

    return LaunchDescription([
        # Set resource paths for Gazebo
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(spider_warehouse, "worlds"),
        ),
        AppendEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=join(spider_warehouse, "models"),
        ),

        # Declare launch arguments
        DeclareLaunchArgument("world_file", default_value=world_file),

        # Launch Gazebo with your world
        gz_sim,

        # Publish robot description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Spawn robot in Ignition
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "spider_description",
                "-topic", "robot_description",
                "-allow_renaming", "true",
                "-x", "-2.0499",
                "-y", "-5.969", 
                "-z", "0.16",
                "-R", "0.0",
                "-P", "0.0",
                "-Y", "1.57"
            ],
            output="screen",
        ),

        # Start ros2_control with controllers yaml
        Node(
           package='controller_manager',
           executable='ros2_control_node',
           parameters=[controllers_param],
           output='screen'
        ),

        # Spawn joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Spawn position_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Joint state publisher (non-GUI)
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[controllers_param],
        ),

        # Optional GUI for debugging joints
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
    ])
