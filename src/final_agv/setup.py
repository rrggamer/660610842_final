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
		'final_teleop = final_agv.teleop_agv:main'
        ],
    },
)
