from setuptools import setup
import os
from glob import glob

package_name = 'valve_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'gpiozero'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple ROS 2 valve control package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'valve_node = valve_pkg.valve_node:main',
            'valve_controller = valve_pkg.valve_controller:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
)
