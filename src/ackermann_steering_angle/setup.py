from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ackermann_steering_angle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boxing',
    maintainer_email='clibang2022@163.com',
    description='通过 TF 变换测量阿克曼车辆前轮实际转向角度',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_steering_angle = ackermann_steering_angle.ackermann_steering_angle:main',
        ],
    },
)
