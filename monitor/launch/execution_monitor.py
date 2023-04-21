from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

import os

def get_execution_monitor_node(context):

    config_file = LaunchConfiguration(
        'config_file').perform(context)

    drone_id = LaunchConfiguration('drone_id')

    parameters=[{
        "drone_id": drone_id
    }]
    
    ns = 'mutac/drone' + drone_id.perform(context)

    parameters.append(config_file)

    execution_monitor_node = Node(
        package='monitor',
        namespace=ns,
        executable='execution_monitor',
        name='execution_monitor',
        parameters=parameters,
        output='screen',
    )

    return [execution_monitor_node]

def generate_launch_description():
    launch_description = LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('config_file',default_value=''),
        OpaqueFunction(function=get_execution_monitor_node)
    ])

    return launch_description
