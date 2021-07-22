import time
import threading
import math
import traceback

from sensor_msgs.msg import LaserScan
import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from .driver import YDLidar
from .parser import YDLidarScanParser
from .logging_rclpy import *

class YDLidarNode(Node):
    def __init__(self):
        super().__init__("ydlidar")
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        self.declare_parameter("device", "/dev/ttyUSB0", ParameterDescriptor(read_only=True))
        # The lidar has autobaud so any baud rate is OK
        # Rates higher than 115200 seems to have no effect
        self.declare_parameter("baudrate", 115200, ParameterDescriptor(read_only=True))
        device = self.get_parameter("device").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.lidar = YDLidar(device, baudrate)
        log_info(f"Device: {device} {baudrate}")
        self.parser = YDLidarScanParser()
        self.motor_hz = self.lidar.get_motor_hz()
        self.declare_parameter("motor_hz", self.motor_hz)
        log_info(f"Motor Hz: {self.motor_hz}")
        self.laser_hz = self.lidar.get_laser_hz()
        self.declare_parameter("laser_hz", self.laser_hz)
        log_info(f"Laser Hz: {self.laser_hz}")
        self.status_interval = 1
        self.last_status = time.time()
        self.points_received = 0
        self.last_end = 0
        self.one_msg_per_rev = True
        self.saved_outputs = []
        self.declare_parameter("msg_frame_id", "map")
        self.msg_frame_id = self.get_parameter("msg_frame_id").get_parameter_value().string_value
    def update_params(self):
        new_motor_hz = self.get_parameter("motor_hz").get_parameter_value().double_value
        new_laser_hz = self.get_parameter("laser_hz").get_parameter_value().integer_value
        if new_motor_hz != self.motor_hz:
            self.lidar.set_motor_hz(new_motor_hz)
        if new_laser_hz != self.laser_hz:
            self.lidar.set_laser_hz(new_laser_hz)
        self.msg_frame_id = self.get_parameter("msg_frame_id").get_parameter_value().string_value
    def spin_once(self):
        if time.time() - self.last_status > self.status_interval:
            log_info(f"{self.points_received} points per second")
            self.points_received = 0
            self.last_status = time.time()
        input_byte = next(self.lidar.read_byte())
        if len(input_byte) != 1:
            log_warning("Timeout from serial")
            return
        if self.parser.state == self.parser.state.FINDING_HEADER:
            log_debug3(f"HEAD {input_byte}")
        self.parser.run_once(input_byte)
        if not self.parser.output_ready():
            return
        output = self.parser.read_output()
        if not self.one_msg_per_rev:
            # Send this one output only
            self.send_msg(output)
            self.update_params()
            return
        else:
            # Check if its a new rev
            if not self.parser.new_rev:
                self.saved_outputs += [output]
                return
            # See if there is output to send
            if len(self.saved_outputs) == 0:
                self.parser.new_rev = False
                self.saved_outputs = []
                return
            # Assemble real output
            output2 = [None] * 4
            output2[0] = self.saved_outputs[0][0]
            output2[1] = self.saved_outputs[-1][0]
            output2[3] = [x[3] for x in self.saved_outputs]
            output2[3] = [x for y in output2[3] for x in y]
            log_debug(f"Points per revolution: {len(output2[3])}")
            log_debug2(f"Gaps: {[b[0] - a[1] for a, b in zip(self.saved_outputs[:-1], self.saved_outputs[1:])]}")
            if output2[1] < output2[0]:
                output2[1] += 360
            output2[2] = (output2[1] - output2[0]) / (len(output2[3]) - 1)
            self.send_msg(output2)
            # Reset flag
            self.parser.new_rev = False
            self.saved_outputs = []
            self.update_params()
            return
    def send_msg(self, output):
        msg = LaserScan()
        msg.header.frame_id = self.msg_frame_id
        msg.header.stamp.sec = int(self.parser.packet_start)
        self.last_end = output[1]
        msg.angle_min = math.radians(output[0])
        msg.angle_max = math.radians(output[1])
        msg.angle_increment = math.radians(output[2])
        msg.time_increment = 1 / self.laser_hz
        msg.scan_time = 1 / self.motor_hz
        msg.range_min = 0.1
        msg.range_max = 16.0
        # Output from lidar is in mm but ROS expects m
        msg.ranges = [o / 1000 for o in output[3]]
        self.points_received += len(msg.ranges)
        self.pub.publish(msg)

def node_main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("ydlidar")
    node = YDLidarNode()
    # The serial port saturates at around 4700 Hz
    # Faster Hz leads to dropped samples
    node.lidar.set_laser_hz(4000)
    # Set motor Hz to as high as possible
    node.lidar.set_motor_hz(10)
    rclpy_spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    rclpy_spin_thread.start()
    with node.lidar:
        while rclpy.ok():
            node.spin_once()
    node.destroy_node()
    rclpy.shutdown()
    
# main() is called directly by ROS2
def main(args=None):
    try:
        node_main(args=args)
    except:
        traceback.print_exc()
        log_info("Shutting down")
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()