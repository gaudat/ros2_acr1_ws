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

echo 'Running remote control node...'

python3 -m apm_driver.teleop_node &

echo 'Ready'
echo 'Check if the power switch is on'

while ! /home/pren/check_power.sh
do sleep 5s
echo 'Charging'
done

# Kill all nodes
jobs -p | xargs kill

echo 'End'

sudo poweroff
