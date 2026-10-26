"""Pi link framing, the PC/Pi side of firmware/Core/Src/link_codec.c.

A frame on the wire:

    0x00  COBS( type | seq | payload | crc_lo | crc_hi )  0x00

- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR)
  over type | seq | payload, low byte first. binascii.crc_hqx(data, 0xFFFF)
  is exactly that CRC.
- COBS removes every 0x00 from the body, so 0x00 only ever means "frame
  boundary"; a receiver that joins mid-stream is in step at the next 0x00.
- The body (type + seq + payload + CRC) is at most 124 bytes, so the COBS
  bytes between two delimiters are at most 125.

Both sides are checked against the same vectors,
firmware/tests/link_vectors.txt (tools/test_rover_link.py and
firmware/tests/test_link_codec.c). No dependencies beyond the standard
library, so a ROS 2 node can import this file as it is.
"""
import binascii
import struct
from dataclasses import dataclass

MAX_PAYLOAD = 120
BODY_OVERHEAD = 4                         # type + seq + CRC
MAX_BODY = MAX_PAYLOAD + BODY_OVERHEAD    # 124
MAX_COBS = MAX_BODY + 1                   # 125
MAX_WIRE = MAX_COBS + 2                   # 127, both delimiters included

# LinkFrameStatus names, as in link_codec.h (the vector file uses them).
OK = 'OK'
COBS_ERROR = 'COBS_ERROR'
TOO_SHORT = 'TOO_SHORT'
TOO_LONG = 'TOO_LONG'
CRC_ERROR = 'CRC_ERROR'
# Stream-only: the bytes before the first delimiter seen (joined mid-frame,
# or text from a board still in text mode). Not an error.
LEADING = 'LEADING'


class FrameError(ValueError):
    """A piece between two delimiters that isn't a valid frame; .status is
    one of COBS_ERROR, TOO_SHORT, TOO_LONG, CRC_ERROR."""

    def __init__(self, status):
        super().__init__(status)
        self.status = status


@dataclass(frozen=True)
class Frame:
    type: int
    seq: int
    payload: bytes


def crc16(data):
    """CRC-16/CCITT-FALSE of data (bytes-like)."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def cobs_encode(data):
    """COBS of data, without delimiters."""
    out = bytearray([0])
    code_index = 0
    code = 1
    length = len(data)
    for i, byte in enumerate(data):
        if byte:
            out.append(byte)
            code += 1
        # A 0x00 ends the block; so do 254 data bytes, but then a new block
        # is opened only if more input follows.
        if not byte or (code == 0xFF and i + 1 < length):
            out[code_index] = code
            code = 1
            code_index = len(out)
            out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data):
    """Decodes the bytes between two delimiters. Raises FrameError(COBS_ERROR)
    if they are empty, contain a 0x00, or a code runs past the end."""
    if not data:
        raise FrameError(COBS_ERROR)
    out = bytearray()
    read = 0
    length = len(data)
    while read < length:
        code = data[read]
        read += 1
        if code == 0 or read + code - 1 > length:
            raise FrameError(COBS_ERROR)
        block = data[read:read + code - 1]
        if 0 in block:
            raise FrameError(COBS_ERROR)
        out += block
        read += code - 1
        if code != 0xFF and read < length:
            out.append(0)
    return bytes(out)


def encode_frame(frame_type, seq, payload=b''):
    """A whole frame, both delimiters included."""
    if not 0 <= frame_type <= 0xFF or not 0 <= seq <= 0xFF:
        raise ValueError('type and seq are single bytes')
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload of {len(payload)} bytes, at most {MAX_PAYLOAD}')
    body = bytes([frame_type, seq]) + bytes(payload)
    crc = crc16(body)
    body += bytes([crc & 0xFF, crc >> 8])
    return b'\x00' + cobs_encode(body) + b'\x00'


def decode_frame(cobs_bytes):
    """Checks the bytes between two delimiters; returns a Frame or raises
    FrameError, in the same order of checks as LinkFrame_Decode."""
    if len(cobs_bytes) > MAX_COBS:
        raise FrameError(TOO_LONG)
    body = cobs_decode(cobs_bytes)
    if len(body) < BODY_OVERHEAD:
        raise FrameError(TOO_SHORT)
    received = body[-2] | (body[-1] << 8)
    if crc16(body[:-2]) != received:
        raise FrameError(CRC_ERROR)
    return Frame(body[0], body[1], bytes(body[2:-2]))


class FrameDecoder:
    """Splits a byte stream into frames. feed() takes chunks exactly as read
    from the port and returns one (status, frame, raw) tuple per finished
    piece: status OK with a Frame, a FrameError status with None, or LEADING
    for the bytes before the first delimiter. Empty pieces (two 0x00 in a
    row) are skipped. A piece is never kept beyond MAX_COBS bytes: the rest
    of it is dropped and it ends as TOO_LONG, so text or noise can't grow the
    buffer."""

    def __init__(self):
        self.buffer = bytearray()
        self.overlong = False
        self.synced = False
        self.counts = {OK: 0, COBS_ERROR: 0, TOO_SHORT: 0, TOO_LONG: 0, CRC_ERROR: 0, LEADING: 0}

    def feed(self, chunk):
        results = []
        for byte in chunk:
            if byte != 0:
                if len(self.buffer) < MAX_COBS:
                    self.buffer.append(byte)
                else:
                    self.overlong = True
                continue
            raw = bytes(self.buffer)
            overlong = self.overlong
            self.buffer.clear()
            self.overlong = False
            if not self.synced:
                self.synced = True
                if raw or overlong:
                    results.append(self._count(LEADING, None, raw))
                continue
            if not raw and not overlong:
                continue
            if overlong:
                results.append(self._count(TOO_LONG, None, raw))
                continue
            try:
                results.append(self._count(OK, decode_frame(raw), raw))
            except FrameError as error:
                results.append(self._count(error.status, None, raw))
        return results

    def _count(self, status, frame, raw):
        self.counts[status] += 1
        return status, frame, raw


# --- Messages (protocol version 1; firmware/Core/Inc/link_protocol.h) -------------
#
# Fixed payload length per type, little-endian. Packing saturates each field
# to its wire type exactly as the firmware does (an out-of-range value is
# pinned, never wrapped), so the same fields give the same bytes on both
# sides (firmware/tests/link_message_vectors.txt).

PROTOCOL_VERSION = 1

HELLO = 0x01
DRIVE = 0x10
ESTOP = 0x11
HELLO_ACK = 0x81
STATE = 0x82
STATUS = 0x83
MESSAGE_NAMES = {HELLO: 'HELLO', DRIVE: 'DRIVE', ESTOP: 'ESTOP', HELLO_ACK: 'HELLO_ACK', STATE: 'STATE',
                 STATUS: 'STATUS'}

DRIVE_STEER_VALID = 0x01
DRIVE_ARM = 0x02
DRIVE_RESERVED = 0xFC

CPS_PER_UNIT = 50
MIN_SPEED_CPS = 1000
MAX_SPEED_CPS = 5000
MAX_STEER_PERCENT = 100

# HELLO_ACK build_flags: the Pi must refuse to drive a build with any bit set
# (test builds, and a bench build that accepts m/v/s over USB in text mode).
BUILD_FLAG_NAMES = ((0x01, 'IWDG_HANG_TEST'), (0x02, 'SPEED_CAPTURE'), (0x04, 'ENCODER_FAULT_TEST'),
                    (0x08, 'LOOP_TIMING_PROFILE'), (0x10, 'BENCH_TEXT_COMMANDS'))
# STATE flags.
ENC_VALID_NOW = 0x01
ENC_INVALID_IN_PERIOD = 0x02
ENC_READ_FAILED = 0x04
IMU_OK = 0x01
IMU_READ_FAILED = 0x02
# STATUS mag_flags.
MAG_DETECTED = 0x01
MAG_TOO_WEAK = 0x02
MAG_TOO_STRONG = 0x04

# MotorLatchReason (motor_control.h), in code order; tools/test_rover_link.py checks it.
LATCH_REASON_NAMES = ('none', 'BOOT', 'WATCHDOG', 'STALL LOCKED', 'STALL SATURATION', 'ESTIMATE INVALID', 'ESTOP',
                      'HELLO')
# LinkStatusCounter (link_protocol.h), in wire order; tools/test_rover_link.py checks it.
STATUS_COUNTER_NAMES = ('watchdog_trips', 'latches', 'locked_stalls', 'saturation_stalls', 'enc_bad', 'i2c_errors',
                        'imu_errors', 'overruns', 'resyncs', 'tx_dropped', 'steering_timeouts', 'rx_ok',
                        'rx_crc_errors', 'rx_frame_errors', 'rx_rejected', 'rx_seq_gaps', 'rx_duplicates',
                        'rx_uart_errors', 'rx_ring_drops', 'rx_text_ignored')

HELLO_ACK_FIELDS = ('version', 'build_flags', 'reset_flags', 'imu_ok', 'imu_who', 'min_cps', 'max_cps',
                    'cps_per_unit', 'state_period_ms', 'status_period_ms')
STATE_FIELDS = ('t_ms', 'count', 'speed_cps', 'enc_flags', 'odometer_epoch', 'gyro_x', 'gyro_y', 'gyro_z',
                'accel_x', 'accel_y', 'accel_z', 'imu_samples', 'imu_flags', 'duty', 'motor_state', 'control_state',
                'drive_seq_echo')
STATUS_FIELDS = ('t_ms', 'setpoint_cps', 'integrator_ticks', 'steer_percent', 'steer_pulse_us', 'steer_state',
                 'motor_state', 'control_state', 'fault_reason', 'latch_reason', 'battery_state', 'imu_ok',
                 'mag_flags', 'agc', 'reset_flags', 'vbat_mv', 'vdda_mv', 'imu_temperature', 'magnitude_min',
                 'magnitude_max', 'loop_us_max', 'i2c_us_max', 'spi_us_max') + STATUS_COUNTER_NAMES

_U8, _I8, _U16, _I16 = (0, 0xFF), (-0x80, 0x7F), (0, 0xFFFF), (-0x8000, 0x7FFF)
_HELLO_ACK_STRUCT = struct.Struct('<BBBBBHHHBB')
_STATE_STRUCT = struct.Struct('<IihBB3i3iBBhBB')
_STATUS_STRUCT = struct.Struct('<IhhbHBBBBBBBBBBHHhHHHHH20I')
_DRIVE_STRUCT = struct.Struct('<hbB')
LENGTHS = {HELLO: 1, DRIVE: _DRIVE_STRUCT.size, ESTOP: 0, HELLO_ACK: _HELLO_ACK_STRUCT.size,
           STATE: _STATE_STRUCT.size, STATUS: _STATUS_STRUCT.size}


def _sat(value, limits):
    return max(limits[0], min(limits[1], int(value)))


def _u32(value):
    return int(value) & 0xFFFFFFFF


def _i32(value):
    value = int(value) & 0xFFFFFFFF
    return value - (1 << 32) if value & 0x80000000 else value


def cps_to_units(cps):
    """The firmware's conversion (LinkProtocol_CpsToUnits): |cps| < 1000 -> 0,
    else nearest, ties toward zero, the same for both signs."""
    magnitude = abs(int(cps))
    if magnitude < MIN_SPEED_CPS:
        return 0
    units = (magnitude + CPS_PER_UNIT // 2 - 1) // CPS_PER_UNIT
    return -units if cps < 0 else units


def pack_hello(version=PROTOCOL_VERSION):
    return bytes([version & 0xFF])


def pack_drive(speed_cps, steer_percent=0, steer_valid=False, arm=False, extra_flags=0):
    """A DRIVE payload, saturated to i16 / i8 like the firmware's packer (no
    range check: the firmware does that; see drive_check)."""
    flags = (DRIVE_STEER_VALID if steer_valid else 0) | (DRIVE_ARM if arm else 0) | extra_flags
    return _DRIVE_STRUCT.pack(_sat(speed_cps, _I16), _sat(steer_percent, _I8), _sat(flags, _U8))


def drive_check(payload):
    """What the firmware's LinkProtocol_ParseDrive says about a DRIVE payload:
    'OK' or the LinkParseResult name (BAD_LENGTH, BAD_FLAGS, OUT_OF_RANGE,
    ARM_WITH_SPEED)."""
    if len(payload) != LENGTHS[DRIVE]:
        return 'BAD_LENGTH'
    speed, steer, flags = _DRIVE_STRUCT.unpack(payload)
    if flags & DRIVE_RESERVED:
        return 'BAD_FLAGS'
    if abs(speed) > MAX_SPEED_CPS or abs(steer) > MAX_STEER_PERCENT:
        return 'OUT_OF_RANGE'
    if flags & DRIVE_ARM and speed != 0:
        return 'ARM_WITH_SPEED'
    return 'OK'


def unpack_drive(payload):
    speed, steer, flags = _DRIVE_STRUCT.unpack(payload)
    return {'speed_cps': speed, 'steer_percent': steer, 'steer_valid': bool(flags & DRIVE_STEER_VALID),
            'arm': bool(flags & DRIVE_ARM)}


def pack_hello_ack(fields):
    return _HELLO_ACK_STRUCT.pack(
        PROTOCOL_VERSION, _sat(fields['build_flags'], _U8), _sat(fields['reset_flags'], _U8),
        1 if fields['imu_ok'] else 0, _sat(fields['imu_who'], _U8), MIN_SPEED_CPS, MAX_SPEED_CPS, CPS_PER_UNIT,
        _sat(fields['state_period_ms'], _U8), _sat(fields['status_period_ms'], _U8))


def pack_state(fields):
    f = fields
    return _STATE_STRUCT.pack(
        _u32(f['t_ms']), _i32(f['count']), _sat(f['speed_cps'], _I16), _sat(f['enc_flags'], _U8),
        int(f['odometer_epoch']) & 0xFF, *(_i32(f[k]) for k in ('gyro_x', 'gyro_y', 'gyro_z')),
        *(_i32(f[k]) for k in ('accel_x', 'accel_y', 'accel_z')), _sat(f['imu_samples'], _U8),
        _sat(f['imu_flags'], _U8), _sat(f['duty'], _I16),
        _sat(f['motor_state'], (0, 15)) | (_sat(f['control_state'], (0, 15)) << 4), int(f['drive_seq_echo']) & 0xFF)


def pack_status(fields):
    f = fields
    head = (_u32(f['t_ms']), _sat(f['setpoint_cps'], _I16), _sat(f['integrator_ticks'], _I16),
            _sat(f['steer_percent'], _I8), _sat(f['steer_pulse_us'], _U16), _sat(f['steer_state'], _U8),
            _sat(f['motor_state'], _U8), _sat(f['control_state'], _U8), _sat(f['fault_reason'], _U8),
            _sat(f['latch_reason'], _U8), _sat(f['battery_state'], _U8), 1 if f['imu_ok'] else 0,
            _sat(f['mag_flags'], _U8), _sat(f['agc'], _U8), _sat(f['reset_flags'], _U8), _sat(f['vbat_mv'], _U16),
            _sat(f['vdda_mv'], _U16), _sat(f['imu_temperature'], _I16), _sat(f['magnitude_min'], _U16),
            _sat(f['magnitude_max'], _U16), _sat(f['loop_us_max'], _U16), _sat(f['i2c_us_max'], _U16),
            _sat(f['spi_us_max'], _U16))
    return _STATUS_STRUCT.pack(*head, *(_u32(f[name]) for name in STATUS_COUNTER_NAMES))


def _unpack(structure, payload, what):
    if len(payload) != structure.size:
        raise ValueError(f'{what} payload of {len(payload)} bytes, expected {structure.size}')
    return structure.unpack(payload)


def unpack_hello_ack(payload):
    return dict(zip(HELLO_ACK_FIELDS, _unpack(_HELLO_ACK_STRUCT, payload, 'HELLO_ACK')))


def unpack_state(payload):
    values = list(_unpack(_STATE_STRUCT, payload, 'STATE'))
    states = values.pop(14)   # after duty: mstate in the low nibble, ctl in the high one
    values[14:14] = [states & 0x0F, states >> 4]
    return dict(zip(STATE_FIELDS, values))


def unpack_status(payload):
    return dict(zip(STATUS_FIELDS, _unpack(_STATUS_STRUCT, payload, 'STATUS')))


UNPACKERS = {HELLO_ACK: unpack_hello_ack, STATE: unpack_state, STATUS: unpack_status, DRIVE: unpack_drive,
             HELLO: lambda payload: {'version': payload[0]}, ESTOP: lambda payload: {}}


def decode_message(frame):
    """(name, fields) for a frame of a known type with the right length, else
    (name or 'TYPE_0x..', None)."""
    name = MESSAGE_NAMES.get(frame.type, f'TYPE_0x{frame.type:02X}')
    if frame.type not in UNPACKERS or len(frame.payload) != LENGTHS[frame.type]:
        return name, None
    return name, UNPACKERS[frame.type](frame.payload)


def build_flag_text(flags):
    """'' for a normal build, else the test switches that are on ('SPEED_CAPTURE+...')."""
    names = [name for bit, name in BUILD_FLAG_NAMES if flags & bit]
    unknown = flags & ~sum(bit for bit, _ in BUILD_FLAG_NAMES)
    return '+'.join(names + ([f'0x{unknown:02X}?'] if unknown else []))
