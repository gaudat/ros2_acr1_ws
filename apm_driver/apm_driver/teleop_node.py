from std_msgs.msg import String

import rclpy
from rclpy.node import Node

from .cobs import cobs_encode, cobs_decode

import struct
import time

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        
        self.logger = rclpy.logging.get_logger("teleop_node")
        
        self.sub = self.create_subscription(String, "ppm_packets", self.loop, 10)
        self.pub = self.create_publisher(String, "packets_to_apm", 10)
        
    def loop(self, msg):
        buf = msg.data
        buf = buf.encode("cp437")
        buf = cobs_decode(buf)
        buf = struct.unpack("<shhhhhhhh", buf)
        
        outputs = buf[1:9]
        
        left_y = outputs[1]
        right_y = outputs[2]
        
        sw_l = outputs[4]
        sw_r = outputs[5]
            
        reverse = False

        if sw_l > 1700:
            # Left switch down = Reverse
            reverse = True
        if sw_l > 1200 and sw_l < 1700:
            # Left switch center = Gated
            return
        # Left switch up = Normal
        
        # Right switch center = Stop
        outputs = [1500] * 8
        
        out_left_mul = 0.7
        out_right = 75

        if sw_r > 1700:
            # Right switch down = Auto forward
            outputs[1] = 1500 - (out_right * out_left_mul * (-1 if reverse else 1)) # Right forward
            outputs[2] = 1500 + (out_right * (-1 if reverse else 1)) # Left forward
        if sw_r < 1200:
            # Right switch up = Manual
            outputs[2] = (left_y - 1500) * 200 / 500 + 1500
            outputs[1] = (right_y - 1500) * (-200) / 500 + 1500
        
        outputs = [int(o) for o in outputs]
        
        buf = struct.pack("<hhhhhhhh", *outputs)
        buf = cobs_encode(b"OUT"+buf).decode("cp437")
        msg = String()
        msg.data = buf
        self.pub.publish(msg)

        
def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
