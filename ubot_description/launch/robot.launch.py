#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
import os

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='burger',
                          choices=['waffle_pi', 'burger', 'waffle'],
                          description='Turtlebot3 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation time'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot3',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
]

def generate_launch_description():
    pkg_turtlebot3_description = get_package_share_directory('ubot_description')
    xacro_file = os.path.join(
            get_package_share_directory('ubot_description'),
            'urdf',
             'ubot_burger.urdf')

    print(xacro_file)
    namespace = LaunchConfiguration('namespace')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command(['xacro', ' ', xacro_file, ' ',
                                           'gazebo:=ignition', ' ',
                                           'namespace:=', namespace])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

        # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld

    return LaunchDescription(ARGUMENTS + [
        robot_state_publisher,
        joint_state_publisher
    ])