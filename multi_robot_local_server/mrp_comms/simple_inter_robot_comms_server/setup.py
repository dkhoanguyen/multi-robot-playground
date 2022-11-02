from setuptools import setup

package_name = 'simple_inter_robot_comms_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'get_all_team_service_server = simple_inter_robot_comms_server.simple_robot_team_service_server:main'
        ],
    },
)
