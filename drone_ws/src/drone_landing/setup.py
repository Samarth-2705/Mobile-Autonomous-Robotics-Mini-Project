from setuptools import setup
import os
from glob import glob

package_name = 'drone_landing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'platform_controller = drone_landing.platform_controller:main',
            'landing_controller  = drone_landing.landing_controller:main',
            'aruco_detector      = drone_landing.aruco_detector:main',
            'state_machine       = drone_landing.state_machine:main',
        ],
    },
)
