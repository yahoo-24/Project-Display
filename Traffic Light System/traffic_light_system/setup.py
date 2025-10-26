from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'traffic_light_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='yahia',
    maintainer_email='yahia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_service = traffic_light_system.traffic_light_service:main',
            'traffic_light_client = traffic_light_system.traffic_light_client:main',
            'traffic_light_subscriber = traffic_light_system.traffic_light_subscriber:main',
            'traffic_density_publisher = traffic_light_system.traffic_density_publisher:main',
            'bag_recorder = traffic_light_system.bag_recorder:main',
            'traffic_light_visualizer = traffic_light_system.traffic_light_visualizer:main',
        ],
    },
)
