import os
from glob import glob
from setuptools import setup

package_name = 'multi_robot_playground'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name,'config'), glob('config/*.yml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khoa',
    maintainer_email='khoanguyendacdang2198@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robot = multi_robot_playground.spawn_robot:main'
        ],
    },
)