from setuptools import setup

package_name = 'ydlidar_node'

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
    description='YDLidar Driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ydlidar_node = ydlidar_node.ydlidar_node:main'
        ],
    },
)
