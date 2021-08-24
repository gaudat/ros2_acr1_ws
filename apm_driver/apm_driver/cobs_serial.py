import time
import serial
import struct
import threading

# This is different from cobs.py as no padding is performed here
# Behavior is undefined if first byte of buf is 0
def cobs_encode(buf):
    if len(buf) == 0:
        return b"\x00"
    if buf[0] != 0:
        buf = b"\x00" + buf
        return cobs_encode(buf)
    # buf[0] must be 0 here
    assert buf[0] == 0
    a, sep, b = buf[1:].partition(b"\x00")
    return bytes([len(a)+1])+a+cobs_encode(sep + b)

def cobs_decode(buf):
    if buf == b"":
        return b""
    if buf == b"\x00":
        return b""
    out = bytearray(buf[0])
    out[1:] = buf[1:len(out)]
    return out + cobs_decode(buf[len(out):])

class COBSSerial:
    def __init__(self, port):
        self.port = serial.Serial(None, 115200, timeout=1)
        self.port.port = port
        self.subscribers = {}
        self.port_lock = threading.RLock()
        self.read_thread = None
        self.stop = False
    def begin(self):
        self.port.open()
        time.sleep(0.1)
        with self.port_lock:
            self.port.write(range(64)) # Get out of bootloader
        if self.read_thread is None:
            self.read_thread = threading.Thread(
            target=self.read_thread_fun)
        self.read_thread.start()
    def end(self):
        self.stop = True
        self.read_thread.join()
        self.read_thread = None
        self.port.close()
    def get_packet(self):
        with self.port_lock:
            buf = self.port.read_until(b'\x00')
        buf = cobs_decode(buf)
        return buf[1:]
    def read_thread_fun(self):
        while not self.stop:
            buf = self.get_packet()
            tag = buf[:3].decode("cp437")
            if tag in self.subscribers:
                if "__iter__" not in dir(self.subscribers[tag]):
                    # Just call the function itself
                    self.subscribers[tag](buf)
                else:
                    for f in self.subscribers[tag]:
                        f(buf)
    def write(self, payload_bytes):
        buf = cobs_encode(payload_bytes)
        with self.port_lock:
            self.port.write(buf)
            self.port.flush()