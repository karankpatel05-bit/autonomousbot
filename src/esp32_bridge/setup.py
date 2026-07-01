from setuptools import setup
import os

package_name = 'esp32_bridge'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (os.path.join(share_dir, 'launch'), ['launch/esp32_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='robot@auto.local',
    description='ROS 2 USB-serial bridge for ESP32 with 4x N20 encoder motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge_node = esp32_bridge.esp32_bridge_node:main',
        ],
    },
)
