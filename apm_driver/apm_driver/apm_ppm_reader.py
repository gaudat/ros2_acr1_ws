from std_msgs.msg import String

import rclpy
from rclpy.node import Node

from .cobs import cobs_encode, cobs_decode

import struct
import time

class APMPPMReader(Node):
    def __init__(self):
        super().__init__("apm_ppm_reader")
        
        self.logger = rclpy.logging.get_logger("apm_ppm_reader")
        
        self.sub = self.create_subscription(String, "ppm_packets", self.loop, 10)

    def loop(self, msg):
        buf = msg.data
        buf = buf.encode("cp437")
        buf = cobs_decode(buf)
        buf = struct.unpack("<shhhhhhhh", buf)
        print("\t".join(str(x) for x in buf))
        
def main():
    rclpy.init()
    node = APMPPMReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()