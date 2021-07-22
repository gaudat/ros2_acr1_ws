import struct
import enum
import functools
import time
import math

from .logging_rclpy import *

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
                log_debug3(f"BTR {self.bytes_to_read}")
                if self.sample_count <= 1:
                    # Happens
                    log_warning("Too few points")
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
                        log_warning("Bad check digit")
                        log_debug(self.packet)
                        self.packets_error += 1
                        return
                self.packets_received += 1
                self.readings = get_readings(self.packet, self.apply_angle_correction)
                self.match = True
    def output_ready(self):
        return self.match
    def read_output(self):
        self.match = False
        return self.readings
