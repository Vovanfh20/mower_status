import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mower_status'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vladimir',
    maintainer_email='vovagefling08684@gmail.com',
    description='Статус робота-газонокосилки и фильтрация GPS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'status_node = mower_status.status_node:main',
        ],
    },
)
