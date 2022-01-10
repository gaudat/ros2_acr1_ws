from typing import Optional, Tuple
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import threading
import socket
import json
import time
import math
import base64
import numpy as np
import logging

class LDCPLidar:
    """Low level driver for LDCP Lidar"""
    def __init__(self, ip:str, port:int, logger:Optional[logging.Logger]):
        # Logger
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger("ldcp_lidar")

        # IPv4 address of lidar. Default: 192.168.10.160
        self.ip = ip
        # Port number of lidar. Default: 2105
        self.port = port

        # Optional configs
        # Scan frequency of lidar. Default: 15 Hz
        # Will be updated on connection
        self.scan_frequency = 15

        # State variables
        # Socket instance, None if not connected
        self.socket: Optional[socket.socket] = None
        # Receive buffer for socket
        self.socket_rx_buf: bytearray = bytearray()
        # "id" of next JSON-RPC message to be sent
        self.jsonrpc_seq = 0
        # "id" of last JSON-RPC message received
        self.jsonrpc_ack = None

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip, self.port))
        self.socket.settimeout(0)
        self.get_config()
    def disconnect(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
    def get_config(self):
        config_k = "scan.frequency"
        self.jsonrpc_send('settings/get', {"params": {"entry": config_k}})
        frame, ts = self.jsonrpc_recv()
        parse_res = self.parse_recvd_frame(frame)
        self.scan_frequency = parse_res['result']
        self.logger.debug("{} {}".format(config_k, parse_res['result']))    
    def jsonrpc_send(self, method: str, args: dict):
        send = args.copy()
        send['method'] = method
        send['id'] = self.jsonrpc_seq
        send['jsonrpc'] = '2.0'
        send = json.dumps(send)
        # Sanity check
        assert '\r' not in send
        assert '\n' not in send
        # Compact
        send = send.replace(' ','')
        self.logger.debug("JSON-RPC send {}".format(send))
        # Add frame end
        send += '\r\n'
        send = send.encode()
        self.socket.send(send)
        self.jsonrpc_seq += 1
    def jsonrpc_recv(self, timeout:float=1) -> Optional[Tuple[bytearray, float]]:
        start = time.time()
        recv_timestamp = None
        while time.time() - start < timeout:
            try:
                recv_timestamp = time.time()
                self.socket_rx_buf += self.socket.recv(4096)
            except BlockingIOError:
                time.sleep(0)
                continue
            if b'\n' in self.socket_rx_buf:
                # Split by frame
                frame, sep, remaining = self.socket_rx_buf.partition(b'\n')
                self.socket_rx_buf = remaining
                return frame, recv_timestamp
    def parse_recvd_frame(self, frame: bytes):
        fj: dict = json.loads(frame)
        if 'result' in fj:
            new_ack = fj['id']
            self.logger.debug("JSON-RPC result {}".format(fj['result']))
            # Verify ack number
            if self.jsonrpc_ack is None:
                self.jsonrpc_ack = new_ack
                return fj
            if self.jsonrpc_ack + 1 == new_ack:
                self.jsonrpc_ack = new_ack
                return fj
            if self.jsonrpc_ack + 1 > new_ack:
                self.logger.warning("JSON-RPC skipped ids {}".format(new_ack))
                self.jsonrpc_ack = new_ack
                return fj
            if self.jsonrpc_ack + 1 < new_ack:
                self.logger.warning("JSON-RPC duplicate ids {}".format(new_ack))
                return fj
        if 'method' in fj:
            method = fj['method']
            if method == 'notification/laserScan':
                return self.parse_laserscan(fj)
            else:
                self.logger.warning("JSON-RPC unknown method {}".format(method))
                return fj
        self.logger.warning("JSON-RPC unknown frame")
        return frame

    def parse_laserscan(self, frame_json: dict):
        params = frame_json["params"]
        #print("laserScan {}".format(params))
        block_width = 270 / 8
        block_0_start = -(270 / 2) 
        block = params['block']
        angle_min = block_0_start + block * block_width
        angle_max = block_0_start + (block + 1) * block_width
        angle_min = angle_min / 180 * math.pi
        angle_max = angle_max / 180 * math.pi

        scan_time = 1 / self.scan_frequency
        
        # Default values
        range_min = 0.05
        range_max = 30

        # uint16_t little endian, unit = 2mm
        ranges = base64.b64decode(params['layers'][0]['ranges'])
        ranges = np.frombuffer(ranges, dtype=np.uint16)
        ranges = ranges.astype(np.float32)
        # Transform dist = 0 to dist = inf
        #ranges[ranges == 0] = np.nan
        #ranges must be a number and is finite
        # Transform to unit = m
        ranges *= 2
        ranges /= 1000
        #print(ranges)
        # docs say uint8_t but actually uint16_t little endian from 0 to 255
        intensities = base64.b64decode(params['layers'][0]['intensities'])
        intensities = np.frombuffer(intensities, dtype=np.uint8)
        intensities = intensities.astype(np.float32)
        #print(intensities.shape)

        assert ranges.shape == intensities.shape
        # Calculate increments
        angle_increment = block_width / ranges.shape[0]
        time_increment = scan_time / (ranges.shape[0] * 360 / block_width)

        return {
            'msg': 'laserscan',
            'angle_min': angle_min,
            'angle_max': angle_max,
            'angle_increment': angle_increment,
            'time_increment': time_increment,
            'scan_time': scan_time,
            'range_min': range_min,
            'range_max': range_max,
            'ranges': ranges,
            'intensities': intensities
        }

class LDCPNode(Node):
    def __init__(self):
        super().__init__("ldcp_node")
        self.logger = rclpy.logging.get_logger("ldcp_node")
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        self.declare_parameter("ip", "192.168.10.160")
        self.declare_parameter("port", 2105)
        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        self.lidar = LDCPLidar(ip, port, self.logger)
        self.logger.info("Lidar IP: {} Port: {}".format(ip, port))
        self.declare_parameter("msg_frame_id", "map")
        self.msg_frame_id = self.get_parameter("msg_frame_id").get_parameter_value().string_value
    def update_params(self):
        if self.lidar.socket is not None:
            self.lidar.disconnect()
            reconnect = True
        else:
            reconnect = False
        self.lidar.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.lidar.port = self.get_parameter("port").get_parameter_value().integer_value
        self.logger.info("Lidar IP: {} Port: {}".format(self.lidar.ip, self.lidar.port))
        if reconnect:
            self.lidar.connect()
        self.msg_frame_id = self.get_parameter("msg_frame_id").get_parameter_value().string_value
    def setup(self):
        self.lidar.connect()
        self.logger.info("Send startStreaming")
        self.lidar.jsonrpc_send('scan/startStreaming', {})
    def loop(self):
        frame, ts = self.lidar.jsonrpc_recv()
        parse_res = self.lidar.parse_recvd_frame(frame)
        if parse_res is not None:
            self.logger.debug(parse_res)
        if type(parse_res) == dict and parse_res.get('msg') == 'laserscan':
            # Send laser scan
            msg = LaserScan()
            msg.header.frame_id = self.msg_frame_id
            msg.header.stamp.sec = int(ts)
            msg.header.stamp.nanosec = int((ts % 1) * 1e9)
            msg.angle_min = float(parse_res['angle_min'])
            msg.angle_max =  float(parse_res['angle_max'])
            msg.angle_increment = float(parse_res['angle_increment'])
            msg.time_increment = float(parse_res['time_increment'])
            msg.scan_time =  float(parse_res['scan_time'])
            msg.range_min =  float(parse_res['range_min'])
            msg.range_max = float(parse_res['range_max'])
            msg.ranges = parse_res['ranges'].tolist()
            msg.intensities = parse_res['intensities'].tolist()
            self.pub.publish(msg)
    def teardown(self):
        self.logger.info("Send stopStreaming")
        self.lidar.jsonrpc_send('scan/stopStreaming', {})
        frame, ts = self.lidar.jsonrpc_recv()
        parse_res = self.lidar.parse_recvd_frame(frame)
        if parse_res is not None:
            self.logger.debug(parse_res)
        self.lidar.disconnect()
    
def main(args=None):
    rclpy.init(args=args)
    node = LDCPNode()
    node.setup()
    rclpy_spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    rclpy_spin_thread.start()
    try:
        while rclpy.ok():
            node.loop()
    except KeyboardInterrupt:
        pass
    node.teardown()
    node.destroy_node()
    rclpy.shutdown()

def main_test_scan_freq(args=None):
    lidar = LDCPLidar('192.168.10.160', 2105)
    lidar.connect()
    lidar.jsonrpc_send('settings/get', {"params": {"entry": "scan.frequency"}})
    frame, ts = lidar.jsonrpc_recv()
    
    print(frame)

if __name__ == "__main__":
    main()