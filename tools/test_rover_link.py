"""Tests for rover_link.py. Run from the repo root: python3 -m pytest tools/

The vectors in firmware/tests/link_vectors.txt are shared with the
firmware's test (firmware/tests/test_link_codec.c), so the C and Python
codecs can't drift apart. The rest checks the codec against independent
references (a bit-by-bit CRC, exhaustive COBS inputs) and the stream decoder
against corrupted streams.
"""
import itertools
import re
import random
from pathlib import Path

import pytest

import rover_link

VECTOR_FILE = Path(__file__).resolve().parents[1] / 'firmware' / 'tests' / 'link_vectors.txt'


def unhex(text):
    return b'' if text == '-' else bytes.fromhex(text)


def vector_lines(kind):
    lines = [line.split() for line in VECTOR_FILE.read_text(encoding='ascii').splitlines()]
    return [fields[1:] for fields in lines if fields and fields[0] == kind]


def crc_bitwise(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def accepted_frames(wire):
    """Frames a receiver would accept from wire, split at every 0x00."""
    accepted = []
    for piece in bytes(wire).split(b'\x00'):
        if piece:
            try:
                accepted.append(rover_link.decode_frame(piece))
            except rover_link.FrameError:
                pass
    return accepted


# --- Shared vectors ---------------------------------------------------------------

def test_vector_file_has_every_kind():
    assert [len(vector_lines(kind)) for kind in ('crc', 'cobs', 'frame', 'bad')] == [5, 12, 6, 8]


@pytest.mark.parametrize('data, expected', vector_lines('crc'))
def test_crc_vectors(data, expected):
    assert rover_link.crc16(unhex(data)) == int(expected, 16)


@pytest.mark.parametrize('raw, encoded', vector_lines('cobs'))
def test_cobs_vectors(raw, encoded):
    assert rover_link.cobs_encode(unhex(raw)) == unhex(encoded)
    assert rover_link.cobs_decode(unhex(encoded)) == unhex(raw)


@pytest.mark.parametrize('frame_type, seq, payload, wire', vector_lines('frame'))
def test_frame_vectors(frame_type, seq, payload, wire):
    frame_type, seq, payload, wire = int(frame_type, 16), int(seq, 16), unhex(payload), unhex(wire)
    assert rover_link.encode_frame(frame_type, seq, payload) == wire
    assert rover_link.decode_frame(wire[1:-1]) == rover_link.Frame(frame_type, seq, payload)


@pytest.mark.parametrize('status, piece', vector_lines('bad'))
def test_bad_frame_vectors(status, piece):
    with pytest.raises(rover_link.FrameError) as error:
        rover_link.decode_frame(unhex(piece))
    assert error.value.status == status


# --- CRC -------------------------------------------------------------------------

def test_crc_check_value_and_empty():
    assert rover_link.crc16(b'123456789') == 0x29B1
    assert rover_link.crc16(b'') == 0xFFFF


def test_crc_matches_bit_by_bit_reference():
    generator = random.Random(1)
    for byte in range(256):
        assert rover_link.crc16(bytes([byte])) == crc_bitwise([byte])
    for _ in range(500):
        data = bytes(generator.randrange(256) for _ in range(generator.randrange(200)))
        assert rover_link.crc16(data) == crc_bitwise(data)


# --- COBS ------------------------------------------------------------------------

def patterns(length, generator):
    yield bytes(length)
    yield bytes(generator.randrange(1, 256) for _ in range(length))
    yield bytes(0 if generator.random() < 0.3 else generator.randrange(256) for _ in range(length))


def test_cobs_round_trip_no_zeros_and_length():
    generator = random.Random(2)
    for length in range(301):
        for data in patterns(length, generator):
            encoded = rover_link.cobs_encode(data)
            assert 0 not in encoded
            assert rover_link.cobs_decode(encoded) == data
            if length <= 254:
                assert len(encoded) == length + 1
            assert len(encoded) <= length + 1 + length // 254


def test_cobs_decode_is_strict():
    """Every input that decodes is the canonical encoding of its result:
    exhaustive for 1 and 2 bytes, random up to 40 (as in the C test)."""
    decoded_count = 0
    candidates = [bytes([a]) for a in range(256)] + [bytes(pair) for pair in itertools.product(range(256), repeat=2)]
    generator = random.Random(3)
    alphabet = [1, 2, 3, 4, 0]
    for _ in range(50000):
        length = generator.randrange(3, 41)
        candidates.append(bytes(generator.choice(alphabet) if generator.random() < 0.6 else generator.randrange(256)
                                for _ in range(length)))
        # Near-valid: a real encoding with one byte changed.
        near = bytearray(rover_link.cobs_encode(bytes(generator.choice(alphabet) for _ in range(length))))
        near[generator.randrange(len(near))] = generator.randrange(256)
        candidates.append(bytes(near))
    for candidate in candidates:
        try:
            decoded = rover_link.cobs_decode(candidate)
        except rover_link.FrameError as error:
            assert error.status == rover_link.COBS_ERROR
            continue
        decoded_count += 1
        assert rover_link.cobs_encode(decoded) == candidate
    assert decoded_count > 1000


@pytest.mark.parametrize('piece', [b'', b'\x02\x11\x00', b'\x04\x11\x22', b'\xff\x01\x02', b'\x00'])
def test_cobs_rejects(piece):
    with pytest.raises(rover_link.FrameError):
        rover_link.cobs_decode(piece)


# --- Frames ----------------------------------------------------------------------

def test_frame_round_trip_every_length_type_and_seq():
    generator = random.Random(4)
    for length in range(rover_link.MAX_PAYLOAD + 1):
        payload = next(itertools.islice(patterns(length, generator), length % 3, None))
        frame_type, seq = generator.randrange(256), generator.randrange(256)
        wire = rover_link.encode_frame(frame_type, seq, payload)
        assert len(wire) == length + 7 and wire[0] == 0 and wire[-1] == 0 and 0 not in wire[1:-1]
        assert rover_link.decode_frame(wire[1:-1]) == rover_link.Frame(frame_type, seq, payload)
    for value in range(256):
        wire = rover_link.encode_frame(value, 255 - value)
        assert rover_link.decode_frame(wire[1:-1]) == rover_link.Frame(value, 255 - value, b'')
    assert rover_link.MAX_WIRE == 127


def test_frame_refuses_bad_arguments():
    with pytest.raises(ValueError):
        rover_link.encode_frame(1, 0, bytes(rover_link.MAX_PAYLOAD + 1))
    with pytest.raises(ValueError):
        rover_link.encode_frame(256, 0)
    with pytest.raises(ValueError):
        rover_link.encode_frame(1, -1)


def test_frame_length_limits():
    with pytest.raises(rover_link.FrameError) as error:
        rover_link.decode_frame(b'\x01' * (rover_link.MAX_COBS + 1))
    assert error.value.status == rover_link.TOO_LONG
    with pytest.raises(rover_link.FrameError) as error:
        rover_link.decode_frame(b'\x01' * rover_link.MAX_COBS)
    assert error.value.status == rover_link.CRC_ERROR
    with pytest.raises(rover_link.FrameError) as error:
        rover_link.decode_frame(b'\x01' * 4)
    assert error.value.status == rover_link.TOO_SHORT


def test_single_bit_flips_are_never_accepted():
    generator = random.Random(5)
    for trial in range(60):
        length = generator.randrange(rover_link.MAX_PAYLOAD + 1)
        payload = bytes(generator.randrange(256) for _ in range(length))
        wire = bytearray(rover_link.encode_frame(generator.randrange(256), generator.randrange(256), payload))
        for bit in range(len(wire) * 8):
            wire[bit // 8] ^= 1 << (bit % 8)
            assert accepted_frames(wire) == []
            wire[bit // 8] ^= 1 << (bit % 8)


def test_double_bit_flips_in_a_drive_sized_frame_are_never_accepted():
    wire = bytearray(rover_link.encode_frame(0x10, 7, b'\xe8\x03\x9c\x03'))
    bits = len(wire) * 8
    for first, second in itertools.combinations(range(bits), 2):
        wire[first // 8] ^= 1 << (first % 8)
        wire[second // 8] ^= 1 << (second % 8)
        assert accepted_frames(wire) == []
        wire[first // 8] ^= 1 << (first % 8)
        wire[second // 8] ^= 1 << (second % 8)


# --- Stream decoder --------------------------------------------------------------

def test_stream_leading_bytes_then_frames_in_any_chunking():
    frames = [rover_link.Frame(0x82, n, bytes(range(n, n + 42)) if n % 2 else b'\x00' * 42) for n in range(5)]
    stream = b't_ms=100 raw=1 count=2\r\n' + b''.join(rover_link.encode_frame(f.type, f.seq, f.payload) for f in frames)
    for chunk_size in (1, 2, 3, 7, 64, len(stream)):
        decoder = rover_link.FrameDecoder()
        results = []
        for start in range(0, len(stream), chunk_size):
            results += decoder.feed(stream[start:start + chunk_size])
        assert [status for status, _, _ in results] == [rover_link.LEADING] + [rover_link.OK] * 5
        assert [frame for _, frame, _ in results[1:]] == frames
        assert decoder.counts[rover_link.OK] == 5 and decoder.counts[rover_link.LEADING] == 1


def test_stream_overlong_piece_is_bounded_and_resyncs():
    decoder = rover_link.FrameDecoder()
    decoder.feed(b'\x00')
    assert decoder.feed(b'x' * 10000) == []
    assert len(decoder.buffer) <= rover_link.MAX_COBS  # bounded while the piece is still arriving
    results = decoder.feed(rover_link.encode_frame(0x82, 1, b'abc'))
    assert [status for status, _, _ in results] == [rover_link.TOO_LONG, rover_link.OK]
    assert len(decoder.buffer) == 0


def test_stream_with_random_corruption_never_yields_a_frame_that_was_not_sent():
    """Frames interleaved with noise, dropped bytes, inserted bytes and bit
    flips: every frame accepted is byte-identical to one that was sent, and
    the clean ones around the damage still get through."""
    generator = random.Random(6)
    sent = set()
    stream = bytearray()
    clean = 0
    for n in range(3000):
        payload = bytes(generator.randrange(256) for _ in range(generator.randrange(0, 50)))
        frame = rover_link.Frame(generator.randrange(256), n & 0xFF, payload)
        sent.add(frame)
        wire = bytearray(rover_link.encode_frame(frame.type, frame.seq, frame.payload))
        damage = generator.randrange(6)
        if damage == 1:
            wire[generator.randrange(len(wire))] ^= 1 << generator.randrange(8)
        elif damage == 2:
            del wire[generator.randrange(len(wire))]
        elif damage == 3:
            wire.insert(generator.randrange(len(wire)), generator.randrange(256))
        elif damage == 4:
            wire += bytes(generator.randrange(256) for _ in range(generator.randrange(1, 20)))
        else:
            clean += 1
        stream += wire
    decoder = rover_link.FrameDecoder()
    results = decoder.feed(stream)
    accepted = [frame for status, frame, _ in results if status == rover_link.OK]
    assert all(frame in sent for frame in accepted)
    assert len(accepted) >= clean


# --- Messages: shared vectors and the C headers -------------------------------------

MESSAGE_VECTOR_FILE = VECTOR_FILE.with_name('link_message_vectors.txt')
FIRMWARE_INC = VECTOR_FILE.parents[1] / 'Core' / 'Inc'


def message_vectors(kind):
    lines = [line.split() for line in MESSAGE_VECTOR_FILE.read_text(encoding='ascii').splitlines()]
    return [fields[1:] for fields in lines if fields and fields[0] == kind]


def test_message_vector_file_has_every_kind():
    assert [len(message_vectors(kind)) for kind in ('units', 'drive', 'hello_ack', 'state', 'status')] == [37, 19, 3, 3, 2]


@pytest.mark.parametrize('cps, units', message_vectors('units'))
def test_cps_to_units_vectors(cps, units):
    assert rover_link.cps_to_units(int(cps)) == int(units)


def test_cps_to_units_every_value_is_symmetric_and_raw_minimum():
    for cps in range(-rover_link.MAX_SPEED_CPS, rover_link.MAX_SPEED_CPS + 1):
        units = rover_link.cps_to_units(cps)
        assert rover_link.cps_to_units(-cps) == -units
        assert (units == 0) == (abs(cps) < rover_link.MIN_SPEED_CPS)


@pytest.mark.parametrize('vector', message_vectors('drive'), ids=lambda v: '-'.join(v[:2]))
def test_drive_vectors(vector):
    payload, result = unhex(vector[0]), vector[1]
    assert rover_link.drive_check(payload) == result
    if result == 'OK':
        speed, steer, steer_valid, arm = (int(value) for value in vector[2:])
        assert rover_link.unpack_drive(payload) == {'speed_cps': speed, 'steer_percent': steer,
                                                    'steer_valid': bool(steer_valid), 'arm': bool(arm)}
        assert rover_link.pack_drive(speed, steer, bool(steer_valid), bool(arm)) == payload


@pytest.mark.parametrize('vector', message_vectors('hello_ack'), ids=lambda v: v[0])
def test_hello_ack_vectors(vector):
    names = ('build_flags', 'reset_flags', 'imu_ok', 'imu_who', 'state_period_ms', 'status_period_ms')
    fields = dict(zip(names, (int(value) for value in vector[:6])))
    payload = unhex(vector[6])
    assert rover_link.pack_hello_ack(fields) == payload
    decoded = rover_link.unpack_hello_ack(payload)
    assert (decoded['version'], decoded['min_cps'], decoded['max_cps'], decoded['cps_per_unit']) == (1, 1000, 5000, 50)
    assert rover_link.pack_hello_ack(decoded) == payload


@pytest.mark.parametrize('vector', message_vectors('state'), ids=lambda v: v[0])
def test_state_vectors(vector):
    fields = dict(zip(rover_link.STATE_FIELDS, (int(value) for value in vector[:-1])))
    payload = unhex(vector[-1])
    assert rover_link.pack_state(fields) == payload
    assert rover_link.pack_state(rover_link.unpack_state(payload)) == payload   # saturated values round-trip


@pytest.mark.parametrize('vector', message_vectors('status'), ids=lambda v: v[0])
def test_status_vectors(vector):
    fields = dict(zip(rover_link.STATUS_FIELDS, (int(value) for value in vector[:-1])))
    payload = unhex(vector[-1])
    assert rover_link.pack_status(fields) == payload
    assert rover_link.pack_status(rover_link.unpack_status(payload)) == payload


def c_enum_members(header, first, last):
    text = (FIRMWARE_INC / header).read_text(encoding='utf-8')
    start = text.index(first + ' = 0')   # the enum itself, not a mention in a comment
    body = text[start:text.index(last, start) + len(last)]
    return re.findall(r'^\s*(\w+)\s*(?:=\s*\d+)?\s*,?', re.sub(r'/\*.*?\*/', '', body, flags=re.DOTALL), re.MULTILINE)


def test_names_match_the_firmware_enums():
    counters = c_enum_members('link_protocol.h', 'LINK_COUNT_WATCHDOG_TRIPS', 'LINK_COUNT_RX_TEXT_IGNORED')
    assert tuple(name.removeprefix('LINK_COUNT_').lower() for name in counters) == rover_link.STATUS_COUNTER_NAMES
    latches = c_enum_members('motor_control.h', 'MOTOR_LATCH_NONE', 'MOTOR_LATCH_LINK_HELLO')
    assert len(latches) == len(rover_link.LATCH_REASON_NAMES) == 8


def test_types_lengths_and_limits_match_link_protocol_h():
    text = (FIRMWARE_INC / 'link_protocol.h').read_text(encoding='utf-8')

    def define(name):
        return int(re.search(r'#define ' + name + r'\s+(0x[0-9A-Fa-f]+|\d+)u?\b', text).group(1), 0)

    for name, value in (('HELLO', rover_link.HELLO), ('DRIVE', rover_link.DRIVE), ('ESTOP', rover_link.ESTOP),
                        ('HELLO_ACK', rover_link.HELLO_ACK), ('STATE', rover_link.STATE),
                        ('STATUS', rover_link.STATUS)):
        assert define(f'LINK_MSG_{name}') == value
        assert define(f'LINK_{name}_LENGTH') == rover_link.LENGTHS[value]
    assert define('LINK_MAX_SPEED_CPS') == rover_link.MAX_SPEED_CPS
    assert define('LINK_PROTOCOL_VERSION') == rover_link.PROTOCOL_VERSION
    assert (define('LINK_DRIVE_STEER_VALID'), define('LINK_DRIVE_ARM'), define('LINK_DRIVE_RESERVED')) == (1, 2, 0xFC)


def test_decode_message_and_build_flags():
    frame = rover_link.decode_frame(rover_link.encode_frame(rover_link.DRIVE, 3, rover_link.pack_drive(2500, -30, True))[1:-1])
    assert rover_link.decode_message(frame) == ('DRIVE', {'speed_cps': 2500, 'steer_percent': -30,
                                                          'steer_valid': True, 'arm': False})
    assert rover_link.decode_message(rover_link.Frame(0x55, 0, b'')) == ('TYPE_0x55', None)
    assert rover_link.decode_message(rover_link.Frame(rover_link.STATE, 0, b'\x00')) == ('STATE', None)
    assert rover_link.build_flag_text(0) == ''
    assert rover_link.build_flag_text(0x0A) == 'SPEED_CAPTURE+LOOP_TIMING_PROFILE'
    assert rover_link.build_flag_text(0x10) == 'BENCH_TEXT_COMMANDS'
    assert rover_link.build_flag_text(0x30) == 'BENCH_TEXT_COMMANDS+0x20?'
    assert rover_link.build_flag_text(0xE0) == '0xE0?'


def test_build_flag_names_match_link_protocol_h():
    text = (FIRMWARE_INC / 'link_protocol.h').read_text(encoding='utf-8')
    defines = dict((name, int(value, 16)) for name, value in
                   re.findall(r'#define LINK_BUILD_(\w+)\s+(0x[0-9A-Fa-f]+)u', text))
    assert {name: bit for bit, name in rover_link.BUILD_FLAG_NAMES} == defines
