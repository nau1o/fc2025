from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'visualize'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naynn',
    maintainer_email='naynn@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pose_node=visualize.pose_node:main',
            'trajectory_node=visualize.trajectory_node:main',
            
        ],
    },
    
)
