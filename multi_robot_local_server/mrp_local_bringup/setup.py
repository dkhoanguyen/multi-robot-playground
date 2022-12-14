import os
from glob import glob
from setuptools import setup

package_name = 'mrp_local_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'config'), glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khoa',
    maintainer_email='khoa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'activate_motion_planners = mrp_local_bringup.activate_motion_planners:main',
            'lifecycle_manager_client = mrp_local_bringup.lifecycle_manager_client:main'
        ],
    },
)
