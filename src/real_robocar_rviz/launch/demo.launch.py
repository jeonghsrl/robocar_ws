import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf_robocar_path = get_package_share_path('real_robocar_rviz')
    default_rviz_config_path = urdf_robocar_path /'robocarNew.rviz'
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    urdf_file_name = 'robocar.urdf'
    urdf = os.path.join(
        get_package_share_directory('real_robocar_rviz'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]),    
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='real_robocar_rviz',
            executable='robocar_state_pub',
            name='robocar_state_pub',
            output='screen'),

    ])
