import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Use sim time argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Locate and process the Xacro file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot_description.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher node
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Final Launch Description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher
    ])
