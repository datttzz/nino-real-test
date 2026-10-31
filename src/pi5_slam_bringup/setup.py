from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

name = 'pi5_slam_bringup'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        *[('share/' + name + '/' + folder, [p for p in glob(folder + '/*') if Path(p).is_file()])
          for folder in ('launch', 'config', 'urdf', 'rviz')],
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='nguyenbinh-shark', maintainer_email='181845870+nguyenbinh-shark@users.noreply.github.com',
    description='Headless STM32 + RPLidar + SLAM Toolbox bringup for ROS 2 Jazzy.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'demo_sensors = pi5_slam_bringup.demo_sensors:main',
        'doctor = pi5_slam_bringup.doctor:main',
        'scan_deskew = pi5_slam_bringup.scan_deskew:main',
    ]},
)
