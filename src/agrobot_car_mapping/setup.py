from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'agrobot_car_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='13824452827@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mapping = agrobot_car_mapping.custom_mapping:main",
            "map_saver = agrobot_car_mapping.map_saver:main",
            "sequence_map_saver = agrobot_car_mapping.sequence_map_saver:main",
            "lidar_data_relay = agrobot_car_mapping.lidar_data_relay:main",
        ],
    },
)
