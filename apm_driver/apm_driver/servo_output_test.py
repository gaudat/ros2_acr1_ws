from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from .cobs import cobs_encode, cobs_decode

import struct
import time
import sys

class ServoOutputTestNode(Node):
    def __init__(self, config, ondone):
        ch, out_us, duration, freq = config
        if duration is None:
            # Default for duration
            duration = 1
        if freq is None:
            # Default for freq
            freq = 10
        super().__init__("servo_output_test")
        self.ch = ch
        self.out_us = out_us
        self.ondone = ondone
        self.logger = rclpy.logging.get_logger("servo_output_test")
        self.pub = self.create_publisher(String, "packets_to_apm", freq)
        self.loop_timer = self.create_timer(1/freq, self.loop)
        start = self.get_clock().now()
        self.end = start + Duration(seconds=duration)
        if ch == -1:
            print(f"All servos", end='')
        else:
            print(f"Servo {ch}", end='')
        print(f": {out_us} us for {duration:.2f} s ... ", end='', flush=True)

    def loop(self):
        if self.get_clock().now() > self.end:
            # Teardown
            print("Done")
            self.destroy_node()
            self.ondone()
            return
        if self.ch == -1:
            outputs = [self.out_us] * 8
        else:
            outputs = [0] * 8
            outputs[self.ch] = self.out_us
        outputs = [int(o) for o in outputs]
        buf = struct.pack("<hhhhhhhh", *outputs)
        buf = cobs_encode(b"OUT"+buf).decode("cp437")
        msg = String()
        msg.data = buf
        self.pub.publish(msg)

def parse_args(args):
    # $0 <servo channel> <output in microseconds> [duration (float) in seconds] [frequency to send]
    # duration defaults to 1 second
    # frequency defaults to 10 hz
    ch = int(args[1])
    out_us = float(args[2])
    out_us = int(out_us)
    duration = None
    freq = None
    if len(args) >= 4:
        duration = float(args[3])
    if len(args) >= 5:
        freq = float(args[4])
    return (ch, out_us, duration, freq)

def main():
    try:
        config = parse_args(sys.argv)
    except Exception:
        print("Usage: $0 <servo channel (0 - 7), -1 = all> <output in microseconds> [duration in seconds] [frequency in hz]", file=sys.stderr)
        exit(-1)
    done = False
    def done_cb():
        nonlocal done
        done = True
    rclpy.init()
    node = ServoOutputTestNode(config, done_cb)
    try:
        while not done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
