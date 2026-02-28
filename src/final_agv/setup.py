import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final_agv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'models'), glob('model/*.task')),
        (os.path.join('share',package_name, 'srv'), glob('srv/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ppha',
    maintainer_email='phakaewpra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'final_sender = final_agv.agv_sender:main',
		'final_teleop = final_agv.teleop_agv:main',
		'final_tracker = final_agv.gesture_tracker_node:main',
		'final_service = final_agv.collision_srv:main'
        ],
    },
)
