import os
from glob import glob
from setuptools import setup

package_name = 'new_arm_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='nsnn1324@gmail.com',
    description='Lightweight host driver bridging MoveIt to the ESPMax ESP32 firmware.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'arm_bridge = new_arm_driver.arm_bridge:main',
        ],
    },
)
