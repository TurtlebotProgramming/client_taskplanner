import os
from glob import glob
from setuptools import setup

package_name = 'client_taskplanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syu',
    maintainer_email='syu@todo.todo',
    description='client taskplanner',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_fsm = client_taskplanner.client_taskplanner:main',
        ],
    },
)