from setuptools import find_packages, setup
from glob import glob

package_name = 'client_taskplanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Changseok Hyun',
    maintainer_email='hyuncs363@gmail.com',
    description='Turtlebot task planner FSM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_fsm = client_taskplanner.node.fsm:main',
        ],
    },
)
