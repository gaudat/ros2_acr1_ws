import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

import tf_transformations

import serial
import struct
import time
import math

def cobs_decode(buf):
    out = bytearray()
    if len(buf) == 0:
        return out
    in_ptr = 0
    while in_ptr < (len(buf) - 1):
        out += b'\x00'
        frame_len = buf[in_ptr]
        out += buf[in_ptr+1:][:frame_len-1]
        in_ptr += frame_len
    if buf[0] != 1:
        out = out[1:]
    return out

class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, transformation):
        super().__init__('apm_imu_tf2_broadcaster')
        
        self.logger = rclpy.logging.get_logger('logger')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base')

        self.declare_parameter('apm_port', '/dev/ttyACM0')
        self.port = self.get_parameter('apm_port').get_parameter_value().string_value
        
        self.port = serial.Serial(self.port, 115200, timeout = 1)
        
        
        self.br = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.01, self.loop)
        
        self.gyro_zero_loops = 100
        self.gyro_zero_loops_done = 0
        self.gyro_offsets = [0, 0, 0]
        
        self.q_saved = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.q_zero = tf_transformations.quaternion_from_euler(0, 0, 0)
        
        imur_unit = 1
        imur_unit /= 100 # Sample rate = 100 Hz
        imur_unit *= 2000 # Full scale in deg / s
        imur_unit *= 2 # Full scale is +/- instead of one side
        imur_unit /= 65536 # 16-bit ADC
        imur_unit *= math.pi
        imur_unit /= 180 # Degree to radian
        
        self.gyro_unit = imur_unit
        print("Gyro unit:", self.gyro_unit)
        
        # Wait for a few seconds to flush
        port_wait = 5
        wait_start = time.monotonic()
        print("Waiting for board to finish resetting")
        while time.monotonic() - wait_start < port_wait:
            self.port.flushInput()
            time.sleep(0.1)
        
        
        print("Setup done")
        
    def get_imu_reading(self):
        buf = self.port.read_until(b'\x00')
        buf = cobs_decode(buf)
        if len(buf) != 12:
            return None
        imur = struct.unpack("<hhhhhh", buf)
        return imur
    
    def gyro_calibrate(self, imur):
        self.gyro_offsets[0] += imur[3]
        self.gyro_offsets[1] += imur[4]
        self.gyro_offsets[2] += imur[5]
        self.gyro_zero_loops -= 1
        self.gyro_zero_loops_done += 1
    
    def remove_imu_offset(self, imur):
        if self.gyro_zero_loops_done == 0:
            # Prevent division by zero
            return imur
        imur = list(imur)
        imur[3] -= (self.gyro_offsets[0] / self.gyro_zero_loops_done)
        imur[4] -= (self.gyro_offsets[1] / self.gyro_zero_loops_done)
        imur[5] -= (self.gyro_offsets[2] / self.gyro_zero_loops_done)
        return imur
    
    def loop(self):
        imur = self.get_imu_reading()
        if imur is None:
            return
        
        if self.gyro_zero_loops > 0:
            self.gyro_calibrate(imur)
            return
        
        imur = self.remove_imu_offset(imur)
        
        t = TransformStamped()
                
        t.header.frame_id = self.get_parameter('parent_frame').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('child_frame').get_parameter_value().string_value
        
        q = tf_transformations.quaternion_from_euler(
            imur[3] * self.gyro_unit, 
            imur[4] * self.gyro_unit, 
            imur[5] * self.gyro_unit)
        
        #print(str(q))
        
        q_out = tf_transformations.quaternion_multiply(self.q_saved, q)
        n_q_out = (q_out[0]**2+q_out[1]**2+q_out[2]**2+q_out[3]**2)**0.5
        q_out = (q_out[0] / n_q_out, q_out[1] / n_q_out, q_out[2] / n_q_out, q_out[3] / n_q_out)
        
        t.transform.rotation.x = q_out[0]
        t.transform.rotation.y = q_out[1]
        t.transform.rotation.z = q_out[2]
        t.transform.rotation.w = q_out[3]
        
        self.q_saved = q_out
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp.nanosec += 10000000
        if t.header.stamp.nanosec > 999999999:
            t.header.stamp.nanosec -= 100000000
            t.header.stamp.sec += 1
        
        self.br.sendTransform(t)


def main():
    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()