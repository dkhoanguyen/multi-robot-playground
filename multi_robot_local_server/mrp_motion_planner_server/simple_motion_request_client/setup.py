import os
from glob import glob
from setuptools import setup

package_name = 'simple_motion_request_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'config'), glob('config/*.yml')),
        (os.path.join('share', package_name,'sample_paths'), glob('sample_paths/*.csv'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_request = simple_motion_request_client.path_request_client:main'
        ],
    },
)
