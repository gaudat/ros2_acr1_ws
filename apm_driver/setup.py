from setuptools import setup

package_name = 'apm_driver'

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
    maintainer='pren',
    maintainer_email='tony.chiu@pren.co',
    description='APM 2.x Board Driver',
    license='All Rights Reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "imu_tf_test = apm_driver.imu_tf_test:main"
        ],
    },
)
