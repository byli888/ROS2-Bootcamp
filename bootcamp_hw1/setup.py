from setuptools import find_packages, setup

package_name = 'bootcamp_hw1'

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
    description='ros2_bootcamp_hw1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber = bootcamp_hw1.subscriber:main',
            'publisher = bootcamp_hw1.publisher:main',
        ],
    },
)
