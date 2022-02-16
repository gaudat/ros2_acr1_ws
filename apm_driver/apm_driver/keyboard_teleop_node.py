from std_msgs.msg import String

import rclpy
from rclpy.node import Node

from .cobs import cobs_encode, cobs_decode

import struct
import time

from .kbhit import KBHit

class KeyboardWASD:
    def __init__(self):
        self.l_velocity = 50
        self.l_accel_add = 200
        self.l_accel_dec = 200
        self.r_velocity = 50
        self.l_accu = 0
        self.r_now = 0
        self.timeout = 0.2
        self.last_received = None
    def update_w(self, time_delta):
        if self.l_accu >= 0:
            self.l_accu += self.l_accel_add * time_delta
            self.l_accu = min(self.l_accu, self.l_velocity)
        if self.l_accu < 0:
            self.l_accu += self.l_accel_dec * time_delta
            self.l_accu = min(self.l_accu, 0)
    def update_s(self, time_delta):
        if self.l_accu <= 0:
            self.l_accu += -self.l_accel_add * time_delta
            self.l_accu = max(self.l_accu, -self.l_velocity)
        if self.l_accu > 0:
            self.l_accu += -self.l_accel_dec * time_delta
            self.l_accu = max(self.l_accu, 0)
    def update_ws_none(self, time_delta):
        if self.l_accu >= 0:
            self.l_accu += -self.l_accel_dec * time_delta
            self.l_accu = max(self.l_accu, 0)
        if self.l_accu < 0:
            self.l_accu += self.l_accel_dec * time_delta
            self.l_accu = min(self.l_accu, 0)
    def update(self, instruction, time_delta):
        if self.last_received is None:
            self.last_received = time.time()
        if time.time() - self.last_received > self.timeout and instruction is None:
            # Timed out, force zero
            self.l_accu = 0
            self.r_now = 0
            return (1500, 1500)
        if instruction is not None:
            self.last_received = time.time()
        if instruction == 'w':
            self.update_w(time_delta)
        elif instruction == 's':
            self.update_s(time_delta)
        elif instruction == 'a':
            self.r_now = self.r_velocity
        elif instruction == 'd':
            self.r_now = -self.r_velocity
        else:
            self.update_ws_none(time_delta)
            self.r_now = 0
        left = 1500 - self.l_accu + self.r_now
        right = 1500 + self.l_accu + self.r_now
        return left, right

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        
        self.logger = rclpy.logging.get_logger("teleop_node")
        
        self.pub = self.create_publisher(String, "packets_to_apm", 10)
        self.loop_timer = self.create_timer(0.02, self.loop)

        self.kb = KBHit()
        self.wasd = KeyboardWASD()
        
    def loop(self):
        if not self.kb.kbhit():
            kb_input = None
        else:
            kb_input = self.kb.getch()
        left, right = self.wasd.update(kb_input, 0.02)

        outputs = [1500] * 8
        # outputs[1] == right motor, + is forward
        # outputs[2] == left motor, - is forward
        outputs[1] = right
        outputs[2] = left

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
