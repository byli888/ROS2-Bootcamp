from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the empty world file
    world_file = os.path.join(os.path.expanduser('~'), 'gazebo_worlds', 'empty_world.sdf')

    # Path to the TurtleBot3 model URDF file
    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_' + os.environ['TURTLEBOT3_MODEL'] + '.urdf')

    # Include the Gazebo launch file with the empty world
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': world_file}.items()
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'turtlebot3_' + os.environ['TURTLEBOT3_MODEL'],
                '-file', urdf_file,
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
    ])
