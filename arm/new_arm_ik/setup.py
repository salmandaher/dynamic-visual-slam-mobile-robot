import os
from glob import glob
from setuptools import setup

package_name = 'new_arm_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='nsnn1324@gmail.com',
    description='Numerical IK interface (with Tkinter GUI) for the New_arm robotic arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ik_gui_node = new_arm_ik.ik_gui_node:main',
        ],
    },
)
