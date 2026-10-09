from setuptools import find_packages, setup

package_name = 'pika_foot_pedal'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    test_suite='test',
    zip_safe=True,
    maintainer='user2',
    maintainer_email='user2@example.com',
    description='Press-to-toggle Pika foot pedal for both arms',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'foot_pedal_node = pika_foot_pedal.node:main',
    ]},
)
