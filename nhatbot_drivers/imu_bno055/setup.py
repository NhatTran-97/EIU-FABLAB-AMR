from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_bno055'
subParam = package_name + "/params"
subConnector = package_name + "/connectors"
subError = package_name + "/error_handling"
subSensor = package_name + "/sensor"



setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name, subParam,subConnector,subError,subSensor],
               
    data_files= [
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
            
        ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'launch/'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nhat',
    maintainer_email='nhat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'talkerNode_exe = imu_bno055.talker_node:main',
            # 'listenerNode_exe = imu_bno055.listener_node:main', 
            # 'clientController_exe = imu_bno055.turtle_controller:main',
            # 'addition_service = imu_bno055.service_node:main',
            # 'addition_client = imu_bno055.client_node:main',
            'bno055_exe = imu_bno055.main_sensor:main',
         
        ],
    },
)
