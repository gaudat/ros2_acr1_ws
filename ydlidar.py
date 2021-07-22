import serial
import threading
import time
import enum
import struct
import functools
import math

class YDLidar:
    def __init__(self, device: str, baudrate: str):
        self.port = serial.Serial(device, baudrate)
        self.port.timeout = 0.1
        self.port_lock = threading.RLock()
        self.stop_scanning()
        self.buf = b''
        self.running = False
        self.laser_speeds = [4000, 8000, 9000]
        self.motor_hz = None
        self.laser_hz = None
    def send_command(self, cmd_bytes, read_bytes=0):
        with self.port_lock:
            self.port.flush()
            self.port.write(cmd_bytes)
            if read_bytes == 0:
                return
            return self.port.read(read_bytes)
    def start_scanning(self):
        self.send_command(b'\xa5\x60')
        self.running = True
    def stop_scanning(self):
        self.send_command(b'\xa5\x65')
        self.running = False
    def __enter__(self, *args):
        self.start_scanning()
    def __exit__(self, *args):
        self.stop_scanning()
    def read_byte(self):
        while self.running:
            with self.port_lock:
                yield self.port.read(1)
    def get_motor_hz(self):
        if self.motor_hz is not None:
            return self.motor_hz
        ret = self.send_command(b'\xa5\x0d', 11)
        self.motor_hz = int.from_bytes(ret[7:11], 'little') / 100
        return self.motor_hz
    def set_motor_hz(self, target_hz):
        print("Target motor Hz", target_hz)
        print("Old motor Hz", self.get_motor_hz())
        running = self.running
        if running:
            self.stop_scanning()
        now_motor_hz = self.get_motor_hz()
        motor_hz_diff = target_hz - now_motor_hz
        motor_hz_diff = round(motor_hz_diff, 1)
        if motor_hz_diff < 0:
            motor_hz_diff = -motor_hz_diff
            # Slow down
            cmd1 = b'\xa5\x0c'
            cmd0p1 = b'\xa5\x0a'
        elif motor_hz_diff > 0:
            # Speed up
            cmd1 = b'\xa5\x0b'
            cmd0p1 = b'\xa5\x09'
            
        while motor_hz_diff >= 0.1:
            if motor_hz_diff > 1:
                self.send_command(cmd1)
                motor_hz_diff -= 1
            elif 0 < motor_hz_diff < 1:
                self.send_command(cmd0p1)
                motor_hz_diff -= 0.1
            else:
                assert False, "Unreachable code"
        # Remove cached motor_hz
        self.motor_hz = None
        self.get_motor_hz()
        print("New motor Hz", self.get_motor_hz())
        if running:
            self.start_scanning()
    def get_laser_hz(self):
        if self.laser_hz is not None:
            return self.laser_hz
        buf = self.send_command(b'\xa5\xd1', 8)
        self.laser_hz = self.laser_speeds[buf[7]]
        return self.laser_hz
    def set_laser_hz(self, hz):
        running = self.running
        if running:
            self.stop_scanning()
        hz = self.laser_speeds.index(hz)
        with self.port_lock:
            self.port.flush()
            read_hz = -1
            while hz != read_hz:
                self.port.write(b'\xa5\xd0')
                buf = self.port.read(8)
                read_hz = buf[7]
        # Remove cached laser_hz
        self.laser_hz = None
        self.get_laser_hz()
        print("Current laser Hz: {}".format(self.get_laser_hz()))
        if running:
            self.start_scanning()

class ByteStreamMatcher:
    def __init__(self, pattern):
        self.match = False
        self.pattern = pattern
        self.match_progress = 0
    def run(self, input_byte):
        if self.match:
            return self.match
        if input_byte[0] == self.pattern[self.match_progress]:
            self.match_progress += 1
            if self.match_progress == len(self.pattern):
                self.match = True
        else:
            self.match = False
            self.match_progress = 0
        return self.match
    def reset(self):
        self.match = False
        self.match_progress = 0

def get_check_digit(packet):
    packet = struct.unpack("<"+"H"*int(len(packet) / 2), packet)
    packet = list(packet)
    # Check code input array = PH FSA S1 S2 ... [CT LSN] LSA
    real_cc = packet[4]
    packet[4] = 0
    cc = functools.reduce(lambda a, b: a ^ b, packet)
    return cc, real_cc

def get_readings(packet, apply_cor):
    # Try to match LaserScan message as much as possible
    packet = struct.unpack("<"+"H"*int(len(packet) / 2), packet)
    start_angle = (packet[2] >> 1) / 64
    end_angle = (packet[3] >> 1) / 64
    angle_increment = (end_angle - start_angle) / (len(packet[5:]) - 1)
    angles = [start_angle + i * angle_increment for i in range(len(packet[5:]))]
    distances = [d / 4 for d in packet[5:]]
    if not apply_cor:
        if angles[0] - angles[-1] > 180:
            angles[-1] += 360
        angle_increment = (angles[-1] - angles[0]) / (len(distances) - 1)
        return (angles[0], angles[-1], angle_increment, distances)
    # Second level correction
    # Maths seems fucky so disable it for now
    for i in range(len(distances)):
        if distances[i] == 0:
            continue
        angle_cor = math.degrees(math.atan(21.8 * (155.3 - distances[i]) / (155.3 * distances[i])))
        angles[i] += angle_cor
    if angles[0] - angles[-1] > 180:
        angles[-1] += 360
    angle_increment = (angles[-1] - angles[0]) / (len(distances) - 1)
    return (angles[0], angles[-1], angle_increment, distances)
    
class YDLidarScanParser:
    class State(enum.Enum):
        FINDING_HEADER = 0
        READING_HEADER = 1
        READING_DATA = 2
    def __init__(self):
        self.scan_header_match = ByteStreamMatcher(b'\xaa\x55\x00')
        self.packet = b''
        self.packet_start = 0
        self.input_buf = b''
        self.bytes_to_read = 0
        self.match = False
        self.state = type(self).State.FINDING_HEADER
        self.new_rev = False
        self.new_rev_match = ByteStreamMatcher(b'\xaa\x55\x01\x01')
        self.bytes_received = 0
        self.packets_received = 0
        self.packets_error = 0
        self.readings = None
        self.apply_angle_correction = False
        self.skip_check_digit = False
    def update_new_rev(self, input_byte):
        self.new_rev_match.run(input_byte)
        if self.new_rev_match.match:
            self.new_rev = True
            self.new_rev_match.reset()
    def run_once(self, input_byte):
        self.bytes_received += 1
        self.packet += input_byte
        self.update_new_rev(input_byte)
        if self.state == self.state.FINDING_HEADER:
            self.packet_start = None
            self.packet = b''
            if not self.scan_header_match.match:
                self.scan_header_match.run(input_byte)
            if self.scan_header_match.match:
                self.packet_start = time.time()
                self.packet = b'\xaa\x55' + input_byte
                self.state = self.state.READING_HEADER
                self.bytes_to_read = 7
                self.input_buf = b''
        elif self.state == self.state.READING_HEADER:
            if self.bytes_to_read > 0:
                self.input_buf += input_byte
                self.bytes_to_read -= 1
            if self.bytes_to_read == 0:
                assert len(self.input_buf) == 7
                (self.sample_count, self.start_angle, 
                 self.end_angle, self.check_code) = \
                        struct.unpack("<BHHH",self.input_buf)
                self.state = self.state.READING_DATA
                self.bytes_to_read = self.sample_count * 2
                #print("BTR", self.bytes_to_read)
                if self.sample_count <= 1:
                    # Happens
                    print("Too few points")
                    self.packets_error += 1
                    self.scan_header_match.reset()
                    self.state = self.state.FINDING_HEADER
                self.input_buf = b''
        elif self.state == self.state.READING_DATA:
            if self.bytes_to_read > 0:
                self.input_buf += input_byte
                self.bytes_to_read -= 1
            if self.bytes_to_read == 0:
                # Check digit
                self.scan_header_match.reset()
                self.state = self.state.FINDING_HEADER
                if not self.skip_check_digit:
                    cc, real_cc = get_check_digit(self.packet)
                    if cc != real_cc:
                        print("Bad check digit")
                        print(self.packet)
                        #print(time.time(), "Bad packet", hex(cc), hex(real_cc))
                        self.packets_error += 1
                        #raise ValueError("Bad packet")
                        return
                self.packets_received += 1
                self.readings = get_readings(self.packet, self.apply_angle_correction)
                self.match = True
    def run(self, input_gen):
        ...
    def output_ready(self):
        return self.match
    def read_output(self):
        self.match = False
        return self.readings

from sensor_msgs.msg import LaserScan
import rclpy

from rclpy.node import Node

class YDLidarNode(Node):
    def __init__(self, device, baudrate):
        super().__init__("ydlidar")
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        self.lidar = YDLidar(device, baudrate)
        print("Device:", device, baudrate)
        self.parser = YDLidarScanParser()
        self.motor_hz = self.lidar.get_motor_hz()
        print("Motor Hz:", self.motor_hz)
        self.laser_hz = self.lidar.get_laser_hz()
        print("Laser Hz:", self.laser_hz)
        self.status_interval = 1
        self.last_status = time.time()
        self.points_received = 0
        self.last_end = 0
        self.one_msg_per_rev = True
        self.saved_outputs = []
    def spin_once(self):
        if time.time() - self.last_status > self.status_interval:
            print(self.points_received, "points")
            self.points_received = 0
            self.last_status = time.time()
        input_byte = next(self.lidar.read_byte())
        if len(input_byte) != 1:
            print(time.time(), "Timeout from serial")
            return
        #if self.parser.state == self.parser.state.FINDING_HEADER:
        #    print("HEAD", input_byte)
        self.parser.run_once(input_byte)
        if not self.parser.output_ready():
            return
        output = self.parser.read_output()
        if not self.one_msg_per_rev:
            # Send this one output only
            self.send_msg(output)
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
            print("Points per revolution", len(output2[3]))
            print("Gaps", [b[0] - a[1] for a, b in zip(self.saved_outputs[:-1], self.saved_outputs[1:])])
            if output2[1] < output2[0]:
                output2[1] += 360
            output2[2] = (output2[1] - output2[0]) / (len(output2[3]) - 1)
            self.send_msg(output2)
            # Reset flag
            self.parser.new_rev = False
            self.saved_outputs = []
            return
    def send_msg(self, output):
        msg = LaserScan()
        msg.header.frame_id = "map"
        msg.header.stamp.sec = int(self.parser.packet_start)
        #print("GAP {:.2f} {:.2f} {:.2f}".format(output[0], output[1], output[0] - self.last_end))
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

def main(args=None):
    rclpy.init(args=args)
    # The lidar has autobaud so any baud rate is OK
    # Rates higher than 115200 seems to have no effect
    node = YDLidarNode("/dev/ttyUSB0", 115200)
    # The serial port saturates at around 4700 Hz
    # Faster Hz leads to dropped samples
    node.lidar.set_laser_hz(4000)
    node.lidar.set_motor_hz(10)
    rclpy_spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    rclpy_spin_thread.start()
    with node.lidar:
        while rclpy.ok():
            node.spin_once()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
        rclpy.shutdown()