import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths to the RViz configuration and URDF files
    package_dir = get_package_share_directory('bootcamp_turtlesim_vacuum_cleaner')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'turtlebot3_config.rviz')
    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    print(f"RViz config path: {rviz_config_path}")
    print(f"URDF file: {urdf}")

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
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

