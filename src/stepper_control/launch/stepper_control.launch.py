#!/usr/bin/env python3
"""
Launch file for Stepper Motor Control System
Compatible with ROS2 Dashing and Ubuntu 18.04
Launches all stepper control nodes with proper configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for stepper control system"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('stepper_control')
    
    # Declare launch arguments
    azimuth_step_pin_arg = DeclareLaunchArgument(
        'azimuth_step_pin',
        default_value='11',
        description='GPIO pin for azimuth motor step signal'
    )
    
    azimuth_dir_pin_arg = DeclareLaunchArgument(
        'azimuth_dir_pin',
        default_value='13',
        description='GPIO pin for azimuth motor direction signal'
    )
    
    azimuth_enable_pin_arg = DeclareLaunchArgument(
        'azimuth_enable_pin',
        default_value='15',
        description='GPIO pin for azimuth motor enable signal'
    )
    
    elevator_step_pin_arg = DeclareLaunchArgument(
        'elevator_step_pin',
        default_value='16',
        description='GPIO pin for elevator motor step signal'
    )
    
    elevator_dir_pin_arg = DeclareLaunchArgument(
        'elevator_dir_pin',
        default_value='18',
        description='GPIO pin for elevator motor direction signal'
    )
    
    elevator_enable_pin_arg = DeclareLaunchArgument(
        'elevator_enable_pin',
        default_value='22',
        description='GPIO pin for elevator motor enable signal'
    )
    
    step_delay_arg = DeclareLaunchArgument(
        'step_delay',
        default_value='0.001',
        description='Delay between motor steps in seconds'
    )
    
    steps_per_command_arg = DeclareLaunchArgument(
        'steps_per_command',
        default_value='10',
        description='Number of steps per keyboard command'
    )
    
    # Safety parameters
    max_azimuth_position_arg = DeclareLaunchArgument(
        'max_azimuth_position',
        default_value='1000',
        description='Maximum azimuth position in steps'
    )
    
    min_azimuth_position_arg = DeclareLaunchArgument(
        'min_azimuth_position',
        default_value='-1000',
        description='Minimum azimuth position in steps'
    )
    
    max_elevator_position_arg = DeclareLaunchArgument(
        'max_elevator_position',
        default_value='500',
        description='Maximum elevator position in steps'
    )
    
    min_elevator_position_arg = DeclareLaunchArgument(
        'min_elevator_position',
        default_value='-500',
        description='Minimum elevator position in steps'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='100.0',
        description='Maximum velocity in steps per second'
    )
    
    # Create stepper control node
    stepper_node = Node(
        package='stepper_control',
        executable='stepper_node',
        name='stepper_control_node',
        output='screen',
        parameters=[{
            'azimuth_step_pin': LaunchConfiguration('azimuth_step_pin'),
            'azimuth_dir_pin': LaunchConfiguration('azimuth_dir_pin'),
            'azimuth_enable_pin': LaunchConfiguration('azimuth_enable_pin'),
            'elevator_step_pin': LaunchConfiguration('elevator_step_pin'),
            'elevator_dir_pin': LaunchConfiguration('elevator_dir_pin'),
            'elevator_enable_pin': LaunchConfiguration('elevator_enable_pin'),
            'step_delay': LaunchConfiguration('step_delay'),
            'steps_per_command': LaunchConfiguration('steps_per_command'),
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('stepper/status', '/stepper/status'),
            ('stepper/joint_states', '/stepper/joint_states'),
        ]
    )
    
    # Create safety monitor node
    safety_monitor_node = Node(
        package='stepper_control',
        executable='safety_monitor',
        name='safety_monitor_node',
        output='screen',
        parameters=[{
            'max_azimuth_position': LaunchConfiguration('max_azimuth_position'),
            'min_azimuth_position': LaunchConfiguration('min_azimuth_position'),
            'max_elevator_position': LaunchConfiguration('max_elevator_position'),
            'min_elevator_position': LaunchConfiguration('min_elevator_position'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'safety_check_rate': 10.0,
        }],
        remappings=[
            ('stepper/joint_states', '/stepper/joint_states'),
            ('stepper/status', '/stepper/status'),
            ('safety/status', '/safety/status'),
            ('emergency_stop', '/emergency_stop'),
        ]
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        azimuth_step_pin_arg,
        azimuth_dir_pin_arg,
        azimuth_enable_pin_arg,
        elevator_step_pin_arg,
        elevator_dir_pin_arg,
        elevator_enable_pin_arg,
        step_delay_arg,
        steps_per_command_arg,
        max_azimuth_position_arg,
        min_azimuth_position_arg,
        max_elevator_position_arg,
        min_elevator_position_arg,
        max_velocity_arg,
        
        # Nodes
        stepper_node,
        safety_monitor_node,
    ]) 