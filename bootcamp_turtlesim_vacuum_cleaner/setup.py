from setuptools import setup
import os
from glob import glob

package_name = 'bootcamp_turtlesim_vacuum_cleaner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Turtlesim vacuum cleaner script',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vacuum_cleaner_turtlebot = bootcamp_turtlesim_vacuum_cleaner.vacuum_cleaner_turtlebot:main',
        ],
    },
)
