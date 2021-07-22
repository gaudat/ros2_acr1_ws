import serial
import threading
import time

from .logging_rclpy import *

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
            self.port.flush()
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
        log_debug(f"Target motor Hz: {target_hz}")
        log_debug(f"Old motor Hz: {self.get_motor_hz()}")
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
            if motor_hz_diff >= 1:
                self.send_command(cmd1)
                motor_hz_diff -= 1
            elif 0 < motor_hz_diff < 1:
                self.send_command(cmd0p1)
                motor_hz_diff -= 0.1
            else:
                assert False, "Unreachable code"
        self.motor_hz = target_hz
        log_info(f"Set motor Hz to {self.get_motor_hz()}")
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
        self.laser_hz = hz
        log_info(f"Set laser Hz to {self.get_laser_hz()}")
        if running:
            self.start_scanning()
