import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('bootcamp_turtlesim_vacuum_cleaner')
    world_path = '/home/jimmy/ros2_ws/src/bootcamp_turtlesim_vacuum_cleaner/worlds/vacuum_world.sdf'
    turtlebot3_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Print statements
    print(f'Package directory: {package_dir}')
    print(f'World path: {world_path}')
    print(f'Turtlebot3 model path: {turtlebot3_model_path}')
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', f"{turtlebot3_model_path}:${{GAZEBO_MODEL_PATH}}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'
            )]),
            launch_arguments={'world': world_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'
            )])
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', os.path.join(
                    get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_burger', 'model.sdf'
                ),
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-R', '0.0',
                '-P', '0.0',
                '-Y', '0.0'
            ],
            output='screen'
        ),
        Node(
            package='bootcamp_turtlesim_vacuum_cleaner',
            executable='vacuum_cleaner_turtlebot',
            name='vacuum_cleaner_turtlebot',
            output='screen'
        ),
    ])
