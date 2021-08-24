
# COBS but always pad a 0 before the payload

def cobs_encode(buf):
    if len(buf) == 0:
        return b"\x00"
    buf = b"\x00" + buf
    return cobs_encode_low(buf)

def cobs_encode_low(buf):
    if len(buf) == 0:
        return b"\x00"
    # buf[0] must be 0 here
    assert buf[0] == 0
    a, sep, b = buf[1:].partition(b"\x00")
    return bytes([len(a)+1])+a+cobs_encode_low(sep + b)

def cobs_decode(buf):
    return cobs_decode_low(buf)[1:]

def cobs_decode_low(buf):
    if buf == b"":
        return b""
    if buf == b"\x00":
        return b""
    out = bytearray(buf[0])
    out[1:] = buf[1:len(out)]
    return out + cobs_decode_low(buf[len(out):])
