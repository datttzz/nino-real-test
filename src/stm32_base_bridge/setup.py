from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'stm32_base_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='tiendat',
    maintainer_email='tiendat@example.com',
    description='Bridge STM32H723 micro-ROS feedback to Linorobot2 topics.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stm32_base_bridge = stm32_base_bridge.bridge_node:main',
        ],
    },
)
