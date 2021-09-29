from std_msgs.msg import String

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

from .cobs import cobs_encode, cobs_decode

import struct
import time

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        
        self.logger = rclpy.logging.get_logger("teleop_node")
        
        self.sub = self.create_subscription(String, "ppm_packets", self.loop, 10)
        self.pub = self.create_publisher(String, "packets_to_apm", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.quat_saved = None
        
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

        # Read transform
        try:       
            quat = self.tf_buffer.lookup_transform(
                    "base",
                    "map",
                    rclpy.time.Time(),
                    #rclpy.duration.Duration(seconds=1)
                    ).transform.rotation
            quat = [quat.x, quat.y, quat.z, quat.w]
        except TransformException as ex:
            quat = None

        if sw_r > 1700:
            # Auto forward; correct based on quat
            if self.quat_saved is None and quat is not None:
                self.quat_saved = quat
        else:
            # Clear saved quat
            self.quat_saved = None
            
        # Trace print quat
        if quat is not None and self.quat_saved is not None:
            dq = tf_transformations.quaternion_multiply(
                    quat, 
                    tf_transformations.quaternion_inverse(self.quat_saved))
            ea = tf_transformations.euler_from_quaternion(dq)
            dyaw = ea[0]
            # ea[0] is yaw, right is -, left is +
        else:
            dyaw = None

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
        
        out_left_mul = 1
        out_right = 50
        yaw_p = 500

        if sw_r > 1700:
            # Right switch down = Auto forward
            if dyaw is not None:
                yaw_corr = dyaw * yaw_p
                #print(yaw_corr)
            else:
                yaw_corr = 0
            outputs[1] = 1500 - (out_right * out_left_mul * (-1 if reverse else 1)) + yaw_corr # Right forward
            outputs[2] = 1500 + (out_right * (-1 if reverse else 1)) + yaw_corr # Left forward
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
