from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ia368_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrolab',
    maintainer_email='adrolab.unicamp@gmail.com',
    description='Package to be used in the IA368 class',
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	        "kinect_node = yolo_detector.kinect:main",
	        "yolo_node = yolo_detector.yolo_3d_detection:main",
	        "tf_node = yolo_detector.tf_node:main",
	        "dummy_creation_node = yolo_detector.dummy_creation:main",
	        "battery_node = autodocking.battery_node:main",
	        "bumper_and_velocity_node = autodocking.bumper_and_velocity_node:main",
	        "charging_base_node = autodocking.charging_base_node:main",
	        "docking_node = autodocking.docking_node:main",
	        "odom_node = position_control.odom_node:main",
	        "pos_control_node = position_control.position_control_node_students:main",
	        "vel_node = position_control.velocity_node:main",
	        "target_node = position_control.target_node:main",
			"gt_node = yolo_detector.gt_node:main",
            "lidar_node = slam_toolbox.lidar:main",
            "tf_node_slam_toolbox = slam_toolbox.tf_node:main",
            "tf_node_rtabmap_rgb = rtabmap.tf_node:main",
            "kinect_node_rtabmap_rgb = rtabmap.kinect:main",
            "ground_truth_node = nav2.ground_truth_node:main"
        ],
    },
)
