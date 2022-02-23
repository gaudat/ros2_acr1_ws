#!/bin/bash

. /opt/ros/rolling/setup.bash

cd ~/ros_ws/src/apm_driver
echo 'Connecting to robot controller...'

python3 -m apm_driver.apm_serial_node &

if ros2 topic list | grep -q '/ppm_packets'
then
echo '...OK'
else
echo '...Failed'
echo 'Check USB connection and restart computer'
exit -1
fi

python3 -m apm_driver.imu_tf_test &

echo 'Running remote control node...'

python3 -m apm_driver.teleop_node &

echo 'Ready'
