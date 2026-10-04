from setuptools import find_packages, setup

package_name = 'robot_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='ghostone',
    maintainer_email='therevaxio@gmail.com',
    description='Gateway between ROS and external clients.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gateway_node = robot_gateway.gateway_node:main',
        ],
    },
)
