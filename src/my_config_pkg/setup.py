from setuptools import setup
import os
from glob import glob

package_name = 'my_config_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('my_config_pkg/launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('my_config_pkg/config/*.yaml')),
        # ROS metadata
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='you@example.com',
    description='Camera config and launch package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)

