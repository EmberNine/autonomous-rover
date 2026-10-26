#!/usr/bin/env python3
"""Print the locators every DDS participant on a ROS 2 domain announces (SPDP).

Each participant (one per ROS 2 process) multicasts its discovery message
every few seconds to 239.255.0.1:(7400 + 250 x domain). This joins that
group and decodes each participant's GUID prefix, the address the packet
came from, and the unicast / multicast locators it asks to be reached on,
for discovery traffic ("metatraffic") and for user data. A wrong address
here (a Docker bridge, a VPN, WSL's NAT address) is the usual reason for
"the topic is listed but no data arrives".

Stdlib only, no root, runs next to any ROS 2 process (the port is shared).
Seen 2026-10-16 (group 0): WSL mirrored advertised UDPv4 192.168.1.218:17910
/ 17911, the Pi UDPv4 192.168.1.18:17910 / 17911 plus a SHM locator.

Usage:  python3 tools/spdp_sniff.py [domain (42)] [seconds (8)]
"""
import socket
import struct
import sys
import time

MULTICAST_GROUP = '239.255.0.1'
SUBMESSAGE_DATA = 0x15
FLAG_LITTLE_ENDIAN = 0x01
FLAG_INLINE_QOS = 0x02
PID_SENTINEL = 0x0001
PL_CDR_BE, PL_CDR_LE = b'\x00\x02', b'\x00\x03'
LOCATOR_PIDS = {0x002F: 'data unicast', 0x0030: 'data multicast', 0x0031: 'default data unicast',
                0x0032: 'metatraffic unicast', 0x0033: 'metatraffic multicast', 0x0048: 'default data multicast'}
LOCATOR_KINDS = {1: 'UDPv4', 2: 'UDPv6', 4: 'TCPv4', 16: 'SHM'}   # 16: Fast DDS shared memory


def discovery_port(domain):
    """The SPDP multicast port of a domain (RTPS 9.6.1.1: PB 7400, DG 250, d0 0)."""
    return 7400 + 250 * domain


def submessages(packet):
    """(kind, flags, body) of each submessage of an RTPS packet; nothing for anything else.
    Stops at a length that runs past the end (a truncated packet)."""
    if len(packet) < 20 or packet[:4] != b'RTPS':
        return
    position = 20
    while position + 4 <= len(packet):
        kind, flags = packet[position], packet[position + 1]
        length = struct.unpack('<H' if flags & FLAG_LITTLE_ENDIAN else '>H', packet[position + 2:position + 4])[0]
        body = packet[position + 4:position + 4 + length] if length else packet[position + 4:]
        if position + 4 + length > len(packet):
            return
        yield kind, flags, body
        if length == 0:                     # RTPS 9.4.5.1.3: 0 = to the end of the packet
            return
        position += 4 + length


def parameters(data, little):
    """(pid, value) of a parameter list up to PID_SENTINEL; returns (list, bytes used)."""
    found = []
    position = 0
    while position + 4 <= len(data):
        pid, length = struct.unpack('<HH' if little else '>HH', data[position:position + 4])
        position += 4
        if pid == PID_SENTINEL:
            break
        found.append((pid, data[position:position + length]))
        position += length
    return found, position


def locator_text(value, little):
    """'UDPv4 192.168.1.18:17910' from a 24-byte Locator_t (kind, port, 16-byte address)."""
    kind, port = struct.unpack('<iI' if little else '>iI', value[:8])
    address = value[8:24]
    host = socket.inet_ntoa(address[12:16]) if kind in (1, 4) else address.hex()
    return f'{LOCATOR_KINDS.get(kind, kind)} {host}:{port}'


def participant_locators(packet):
    """(guid_prefix_hex, [(what, locator_text)]) for an RTPS packet whose DATA submessages carry
    locator parameters (a participant announcement), else None."""
    found = []
    for kind, flags, body in submessages(packet):
        if kind != SUBMESSAGE_DATA or len(body) < 16:
            continue
        little = bool(flags & FLAG_LITTLE_ENDIAN)
        octets_to_inline_qos = struct.unpack('<H' if little else '>H', body[2:4])[0]
        data = body[4 + octets_to_inline_qos:]
        if flags & FLAG_INLINE_QOS:
            _, used = parameters(data, little)
            data = data[used:]
        if data[:2] not in (PL_CDR_BE, PL_CDR_LE):
            continue
        payload_little = data[:2] == PL_CDR_LE
        for pid, value in parameters(data[4:], payload_little)[0]:
            if pid in LOCATOR_PIDS and len(value) >= 24:
                found.append((LOCATOR_PIDS[pid], locator_text(value, payload_little)))
    if not found:
        return None
    return packet[8:20].hex(), found


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    domain = int(argv[0]) if argv else 42
    seconds = float(argv[1]) if len(argv) > 1 else 8.0
    port = discovery_port(domain)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, 'SO_REUSEPORT'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(('', port))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton(MULTICAST_GROUP) + socket.inet_aton('0.0.0.0'))
    sock.settimeout(0.5)
    print(f'listening on {MULTICAST_GROUP}:{port} (domain {domain}) for {seconds:.0f} s')
    seen = set()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            packet, source = sock.recvfrom(65535)
        except socket.timeout:
            continue
        result = participant_locators(packet)
        if result is None or result[0] in seen:
            continue
        prefix, locators = result
        seen.add(prefix)
        print(f'\nparticipant {prefix}  (packet from {source[0]})')
        for what, text in locators:
            print(f'   {what:24s} {text}')
    if not seen:
        print('no participant announcements received')
    return 0 if seen else 1


if __name__ == '__main__':
    sys.exit(main())
