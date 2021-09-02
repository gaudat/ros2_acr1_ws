from std_msgs.msg import String

import rclpy
from rclpy.node import Node

from .cobs import cobs_encode, cobs_decode

import struct
import time
import math

class APMOutputPublisher(Node):
    def __init__(self):
        super().__init__("apm_output_publisher")
        
        self.logger = rclpy.logging.get_logger("apm_output_publisher")
        
        self.pub = self.create_publisher(String, "packets_to_apm", 10)
        
        self.start = time.monotonic()
        
        self.test_timer = self.create_timer(0.01, self.test)

    def test(self):
        if time.monotonic() - self.start < 6:
            output = 1500 + math.sin((time.monotonic() - self.start) * math.pi) * 100
            
            output = int(output)
            self.logger.info(f"Output {output}")
            buf = struct.pack("<h", output) * 8
            buf = cobs_encode(b"OUT"+buf).decode("cp437")
            msg = String()
            msg.data = buf
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = APMOutputPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
