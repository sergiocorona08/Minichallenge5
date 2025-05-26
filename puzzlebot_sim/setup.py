from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergio Augusto Macias Corona',
    maintainer_email='a01352038@tec.mx',
    description='Puzzlebot_Lidar_edition',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_publisher = puzzlebot_sim.joint_state_publisher:main',
            'puzzlebot_kinematic_model = puzzlebot_sim.puzzlebot_kinematic_model:main',
            'localisation = puzzlebot_sim.localisation:main',
            'move_forward = puzzlebot_sim.move_forward:main',
            'set_point_generator = puzzlebot_sim.set_point_generator:main',
            'controller = puzzlebot_sim.controller:main',
            'bug2 = puzzlebot_sim.bug2:main',

        ],
    },
)
