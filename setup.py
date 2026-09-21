import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur3e_letter_writer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baodz20811',
    maintainer_email='baodz20811@gmail.com',
    description=(
        'Cartesian letter-writing demo for UR3/UR3e in Gazebo, planned and '
        'executed through MoveIt 2.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'letter_writer_node = ur3e_letter_writer.letter_writer_node:main',
        ],
    },
)
