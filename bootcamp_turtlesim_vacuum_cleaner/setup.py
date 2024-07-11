from setuptools import find_packages, setup

package_name = 'bootcamp_turtlesim_vacuum_cleaner'

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
    description='Turtlesim vacuum cleaner script',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vacuum_cleaner = bootcamp_turtlesim_vacuum_cleaner.vacuum_cleaner:main',
        ],
    },
)
