from setuptools import find_packages, setup

package_name = 'bootcamp_turtle_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jimmy',
    maintainer_email='jimmy@todo.todo',
    description='ros2_bootcamp_turtle_mover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtle = bootcamp_turtle_mover.move_turtle:main',
        ],
    },
)
