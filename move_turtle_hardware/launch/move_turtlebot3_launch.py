import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('move_turtle_hardware')
    world_path = '/home/jimmy/ros2_ws/src/move_turtle_hardware/worlds/hardware_world.sdf'
    turtlebot3_model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    goal_x = LaunchConfiguration('goal_x', default='2.0')
    goal_y = LaunchConfiguration('goal_y', default='2.0')
    goal_theta = LaunchConfiguration('goal_theta', default='0.0')

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
            package='move_turtle_hardware',
            executable='move_turtlebot3',
            name='move_turtlebot3',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'goal_x': goal_x},
                {'goal_y': goal_y},
                {'goal_theta': goal_theta}
            ]
        )
    ])
