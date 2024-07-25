from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    goal_x = LaunchConfiguration('goal_x', default='2.0')
    goal_y = LaunchConfiguration('goal_y', default='2.0')
    goal_theta = LaunchConfiguration('goal_theta', default='0.0')
    rviz_config_dir = '/home/jimmy/ros2_ws/src/move_turtle_hardware/rviz/turtlebot3_config.rviz'

    return LaunchDescription([
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
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
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        )
    ])
