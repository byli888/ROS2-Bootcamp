from setuptools import setup, find_packages

package_name = 'move_turtle_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/move_turtlebot3_launch.py',
            'launch/move_turtlebot3_rviz_launch.py'
        ]),
        ('share/' + package_name + '/worlds', ['worlds/hardware_world.sdf']),
        ('share/' + package_name + '/rviz', ['rviz/turtlebot3_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='byli888',
    maintainer_email='bol025@ucsd.edu',
    description='Package to move TurtleBot3 to a predefined goal using ROS2 and Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtlebot3 = move_turtle_hardware.move_turtlebot3:main',
        ],
    },
)
