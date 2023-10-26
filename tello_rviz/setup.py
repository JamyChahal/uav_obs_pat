from setuptools import setup
import os
from glob import glob

package_name = 'tello_rviz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Include urdf files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        #Include rviz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        #Include all launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'lib', package_name), glob('tello_rviz/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        #(os.path.join('share', package_name, 'srv'), glob('mas_tracking/srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jamy',
    maintainer_email='jamy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'timestep = tello_rviz.timestep:main',
            'flash = tello_rviz.flash:main',
        ],
    },
)
