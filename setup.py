from glob import glob
from setuptools import find_packages, setup

package_name = 'smart_parking_barrier'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/barrier', glob('models/barrier/*')),
        ('share/' + package_name + '/models/vehicle', glob('models/vehicle/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/docs', glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IEEE RAS',
    maintainer_email='ieee-ras@example.invalid',
    description='ROS 2 / Gazebo autonomous parking barrier.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'barrier_controller = smart_parking_barrier.barrier_controller:main',
            'vehicle_controller = smart_parking_barrier.vehicle_controller:main',
            'simulation_monitor = smart_parking_barrier.simulation_monitor:main',
        ],
    },
)
