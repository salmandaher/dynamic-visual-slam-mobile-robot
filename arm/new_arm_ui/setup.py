import os
from glob import glob
from setuptools import setup

package_name = 'new_arm_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Web app files, enumerated explicitly so npm's node_modules/ is never
        # swept into the install.
        (os.path.join('share', package_name, 'web'), [
            'web/server.js',
            'web/app.js',
            'web/fk.js',
            'web/index.html',
            'web/style.css',
            'web/package.json',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='nsnn1324@gmail.com',
    description='CLI and web interfaces for the new_arm.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'arm_cli = new_arm_ui.cli:main',
        ],
    },
)
