from setuptools import find_packages, setup


package_name = 'pika_session_manager'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lei',
    maintainer_email='lei@example.com',
    description='Pika recording episode and RealMan reset coordinator.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pika_session_manager = pika_session_manager.node:main',
        ],
    },
)
