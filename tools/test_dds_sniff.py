"""Tests for spdp_sniff.py and rtps_sniff.py. Run from the repo root: python3 -m pytest tools/

Synthetic RTPS packets (built here from the RTPS 2.x wire layout): a
participant announcement with locators, little- and big-endian, with and
without inline QoS; truncated and non-RTPS packets; Ethernet/IPv4/UDP frames
for the packet counter. No network, no root.
"""
import collections
import socket
import struct

import rtps_sniff
import spdp_sniff

PREFIX = bytes.fromhex('010f87770100b78b00000000')


def locator(kind, address, port, little=True):
    return struct.pack('<iI' if little else '>iI', kind, port) + bytes(12) + socket.inet_aton(address)


def parameter(pid, value, little=True):
    return struct.pack('<HH' if little else '>HH', pid, len(value)) + value


def submessage(kind, flags, body):
    return bytes([kind, flags]) + struct.pack('<H' if flags & 1 else '>H', len(body)) + body


def spdp_packet(locators, little=True, inline_qos=False):
    """INFO_TS + one DATA submessage whose serialized payload is a parameter list with locators."""
    plist = b''.join(parameter(pid, locator(kind, address, port, little), little)
                     for pid, kind, address, port in locators) + parameter(0x0001, b'', little)
    payload = (b'\x00\x03' if little else b'\x00\x02') + b'\x00\x00' + plist
    header = struct.pack('<HH' if little else '>HH', 0, 16) + bytes(8) + bytes(8)   # flags, octetsToInlineQos, ids, sn
    qos = (parameter(0x0070, b'\x01\x02\x03\x04', little) + parameter(0x0001, b'', little)) if inline_qos else b''
    flags = (0x01 if little else 0) | 0x04 | (0x02 if inline_qos else 0)
    info_ts = submessage(0x09, 0x01, bytes(8))
    return b'RTPS' + b'\x02\x03\x01\x0f' + PREFIX + info_ts + submessage(0x15, flags, header + qos + payload)


PI_LOCATORS = [(0x0032, 1, '192.168.1.18', 17910), (0x0031, 16, '0.0.0.0', 17911), (0x0031, 1, '192.168.1.18', 17911)]


def test_discovery_port():
    assert spdp_sniff.discovery_port(0) == 7400 and spdp_sniff.discovery_port(42) == 17900


def test_participant_locators_little_endian():
    prefix, found = spdp_sniff.participant_locators(spdp_packet(PI_LOCATORS))
    assert prefix == PREFIX.hex()
    assert found == [('metatraffic unicast', 'UDPv4 192.168.1.18:17910'),
                     ('default data unicast', 'SHM ' + bytes(12).hex() + socket.inet_aton('0.0.0.0').hex() + ':17911'),
                     ('default data unicast', 'UDPv4 192.168.1.18:17911')]


def test_participant_locators_big_endian_and_inline_qos():
    for little, inline_qos in ((False, False), (True, True), (False, True)):
        _, found = spdp_sniff.participant_locators(spdp_packet(PI_LOCATORS[:1], little, inline_qos))
        assert found == [('metatraffic unicast', 'UDPv4 192.168.1.18:17910')], (little, inline_qos)


def test_not_an_announcement():
    assert spdp_sniff.participant_locators(b'hello') is None
    assert spdp_sniff.participant_locators(b'RTPS' + bytes(16) + submessage(0x07, 0x01, bytes(28))) is None  # HEARTBEAT
    packet = spdp_packet(PI_LOCATORS)
    for cut in range(len(packet)):                      # truncated anywhere: never raises
        spdp_sniff.participant_locators(packet[:cut])


def udp_frame(source, source_port, destination, destination_port, payload, protocol=17):
    ip = struct.pack('>BBHHHBBH4s4s', 0x45, 0, 20 + 8 + len(payload), 0, 0, 64, protocol, 0,
                     socket.inet_aton(source), socket.inet_aton(destination))
    return bytes(12) + b'\x08\x00' + ip + struct.pack('>HHHH', source_port, destination_port, 8 + len(payload), 0) + payload


def test_udp_of_and_kinds_text():
    payload = spdp_packet(PI_LOCATORS)
    assert rtps_sniff.udp_of(udp_frame('192.168.1.18', 50168, '192.168.1.218', 17911, payload)) == (
        '192.168.1.18', 50168, '192.168.1.218', 17911, payload)
    assert rtps_sniff.udp_of(udp_frame('1.2.3.4', 1, '5.6.7.8', 2, b'', protocol=6)) is None    # TCP
    assert rtps_sniff.udp_of(b'\x00' * 12 + b'\x86\xdd' + bytes(60)) is None                     # IPv6
    assert rtps_sniff.kinds_text(payload) == 'DATA'                  # INFO_TS left out
    assert rtps_sniff.kinds_text(b'not rtps') == ''


def test_count_row_keeps_only_the_domain_and_peer():
    counts = collections.Counter()
    data = spdp_packet(PI_LOCATORS)
    frames = [(udp_frame('192.168.1.18', 50168, '192.168.1.218', 17911, data), False),
              (udp_frame('192.168.1.18', 50168, '192.168.1.218', 17911, data), False),
              (udp_frame('192.168.1.218', 45807, '192.168.1.18', 17910, data), True),
              (udp_frame('192.168.1.18', 50168, '192.168.1.218', 7411, data), False),     # domain 0
              (udp_frame('10.0.0.9', 50168, '192.168.1.218', 17911, data), False)]        # not the peer
    for frame, outgoing in frames:
        rtps_sniff.count_row(counts, 'eth4', outgoing, frame, 42, peer='192.168.1.18')
    assert counts == {('eth4', 'in ', '192.168.1.18:50168', '192.168.1.218:17911', 'DATA'): 2,
                      ('eth4', 'out', '192.168.1.218:45807', '192.168.1.18:17910', 'DATA'): 1}
    lines = rtps_sniff.report(counts)
    assert lines[0].startswith('eth4  in  192.168.1.18:50168') and lines[0].endswith('   2  DATA')
    assert rtps_sniff.report(collections.Counter()) == ['no DDS packets seen']
