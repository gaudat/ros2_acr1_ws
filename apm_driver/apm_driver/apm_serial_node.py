from .cobs_serial import COBSSerial
from .cobs import cobs_encode, cobs_decode

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys

class APMSerial(Node):
    def __init__(self):
        super().__init__("apm_serial_node")
        
        self.logger = rclpy.logging.get_logger("apm_serial_node")
        
        self.declare_parameter("apm_port", "/dev/ttyACM0")
        self.cs = COBSSerial(
            self.get_parameter("apm_port").get_parameter_value().string_value
        )

        # Maybe factorize the following code so new tags can be used easily

        imu_pub = self.create_publisher(String, "imu_packets", 10)
        ppm_pub = self.create_publisher(String, "ppm_packets", 10)

        self.pubs = {
            "IMU": imu_pub,
            "PPM": ppm_pub
        }

        self.cs.subscribers = {
            "IMU": self.pub_fun,
            "PPM": self.pub_fun
        }

        self.sub = self.create_subscription(String, "packets_to_apm", self.write_packet, 10)

    def write_packet(self, msg):
        buf = msg.data.encode("cp437")
        buf = cobs_decode(buf)
        buf = bytes(buf)
        self.cs.write(buf)

    def __enter__(self):
        self.cs.begin()
        self.logger.info("Entered")

    def __exit__(self, *args):
        self.cs.end()
        self.logger.info("Exited")

    def pub_fun(self, payload):
        tag = payload[:3].decode("cp437")
        if tag not in self.pubs:
            self.logger.warning(f"{tag} not found in list of publishers")
            return
        msg = String()
        buf = cobs_encode(payload[3:])
        msg.data = buf.decode("cp437")
        self.pubs[tag].publish(msg)

def main():
    rclpy.init()
    node = APMSerial()
    try:
        with node:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()