import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Assume the workspace is located at ~/ros2_ws
    workspace_dir = os.path.expanduser('~/ros2_ws')
    package_dir = os.path.join(workspace_dir, 'src', 'bootcamp_turtlesim_vacuum_cleaner')
    world_path = os.path.join(package_dir, 'worlds', 'vacuum_world.sdf')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'turtlebot3_config.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    print(f"RViz config path: {rviz_config_path}")

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        #SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(package_dir, 'models')),
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='bootcamp_turtlesim_vacuum_cleaner',
            executable='vacuum_cleaner_turtlebot',
            name='vacuum_cleaner_turtlebot',
            output='screen',
        ),
    ])
