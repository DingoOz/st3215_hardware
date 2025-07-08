#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="st3215_hardware",
            description="Description package with robot URDF/XACRO files",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="st3215_robot.urdf.xacro",
            description="URDF/XACRO description file with the robot",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="Serial port for ST3215 communication",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate",
            default_value="1000000",
            description="Baudrate for ST3215 communication",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    start_rviz = LaunchConfiguration("start_rviz")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "port:=",
            port,
            " ",
            "baudrate:=",
            baudrate,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Hardware configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("st3215_hardware"),
        "config",
        "st3215_controllers.yaml",
    ])

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Position controller spawner
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager"],
    )

    # Velocity controller spawner (inactive by default)
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager", "--inactive"],
    )

    # Effort controller spawner (inactive by default)
    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "-c", "/controller_manager", "--inactive"],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("st3215_hardware"),
        "rviz",
        "st3215_hardware.rviz",
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    # Delay loading and activation of controllers after `robot_state_publisher`
    delay_joint_state_broadcaster_spawner_after_robot_state_publisher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_pub_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_position_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    delay_velocity_controller_spawner_after_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    delay_effort_controller_spawner_after_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velocity_controller_spawner,
            on_exit=[effort_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_robot_state_publisher,
        delay_position_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_velocity_controller_spawner_after_position_controller_spawner,
        delay_effort_controller_spawner_after_velocity_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)