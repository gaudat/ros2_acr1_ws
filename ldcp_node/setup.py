from setuptools import setup

package_name = 'ldcp_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gaudat',
    maintainer_email='ros@gaudat.name',
    description='ROS2 driver for 3irobotix lidars speaking LDCP',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ldcp_node = ldcp_node.ldcp_node:main'
        ],
    },
)
