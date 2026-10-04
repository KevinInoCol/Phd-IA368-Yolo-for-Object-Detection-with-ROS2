import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolo_pf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KevinInoCol',
    maintainer_email='130864069+KevinInoCol@users.noreply.github.com',
    description='YOLO-based potential field navigation for myRobot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node = yolo_pf.detector_node:main',
            'object_map_node = yolo_pf.object_map_node:main',
            'potential_field_node = yolo_pf.potential_field_node:main',
            'velocity_node = yolo_pf.velocity_node:main',
        ],
    },
)
