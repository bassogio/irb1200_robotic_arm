from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
import os

def generate_launch_description():
    pkg = get_package_share_directory('irb1200_description')
    xacro_file = os.path.join(pkg, 'urdf', 'irb1200_stick.xacro')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        )
    )

    # Robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro', xacro_file]),
            'use_sim_time': True
        }]
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'irb1200'],
        output='screen'
    )

    # Controllers
    load_jsb = Node(package='controller_manager', executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen')
    load_jtc = Node(package='controller_manager', executable='spawner',
                    arguments=['joint_trajectory_controller', '--param-file',
                               PathJoinSubstitution([pkg, 'config', 'irb1200_controllers.yaml'])],
                    output='screen')

    return LaunchDescription([gazebo, robot_state_publisher, spawn_entity,
                              load_jsb, load_jtc])
