import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'pika_teleop_bringup'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config', 'ros'),
            glob('config/ros/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lei',
    maintainer_email='lei@example.com',
    description='Pika RealMan teleop shared configuration and launch.',
    license='Apache-2.0',
)
