import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('bootcamp_turtlesim_vacuum_cleaner')
    world_path = os.path.join(package_dir, 'worlds', 'vacuum_world.sdf')
    turtlebot3_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', f"{turtlebot3_model_path}:${{GAZEBO_MODEL_PATH}}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'
            )]),
            launch_arguments={'world': world_path}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py'
            )]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        Node(
            package='bootcamp_turtlesim_vacuum_cleaner',
            executable='vacuum_cleaner_turtlebot',
            name='vacuum_cleaner_turtlebot',
            output='screen'
        ),
    ])
