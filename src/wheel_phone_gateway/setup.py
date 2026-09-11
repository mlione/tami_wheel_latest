from glob import glob

from setuptools import find_packages, setup


package_name = 'wheel_phone_gateway'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@example.com',
    description='将 /debug/viz 图像通过 WebSocket 转发给手机 App。',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'debug_viz_gateway = wheel_phone_gateway.gateway_node:main',
        ],
    },
)

