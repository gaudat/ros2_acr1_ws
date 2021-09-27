from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apm_driver',
            executable='apm_serial_node',
            name='apm_serial_node'
            ),
        Node(
            package='apm_driver',
            executable='imu_tf_test',
            name='imu_tf_test'
            ),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments=['0','0','0','0','0','0','1','world','map']
        #    )
        ])
