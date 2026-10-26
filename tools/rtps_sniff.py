#!/usr/bin/env python3
"""Count a ROS 2 domain's DDS (RTPS) packets per source, destination and submessage kind.

Sees every IPv4 UDP packet on every interface (Linux AF_PACKET, so it needs
root) and keeps those to or from the domain's ports (7400 + 250 x domain and
the next 100). Prints one row per (interface, in/out, source, destination,
submessage kinds) with a count. Shows at once whether user DATA reaches a
host (topic listed but silent), or whether a process sends nothing at all:
on 2026-10-16 (group 0) a `ros2 topic echo` stuck on the CLI daemon sent
not one packet.

Stdlib only. Usage:  sudo python3 tools/rtps_sniff.py [seconds (10)] [peer_ip] [--domain N (42)]
"""
import collections
import socket
import struct
import sys
import time

from spdp_sniff import discovery_port, submessages

ETH_P_ALL = 0x0003
ETHERTYPE_IPV4 = b'\x08\x00'
PACKET_OUTGOING = 4
KIND_NAMES = {0x15: 'DATA', 0x16: 'DATA_FRAG', 0x07: 'HEARTBEAT', 0x06: 'ACKNACK', 0x09: 'INFO_TS',
              0x0E: 'INFO_DST', 0x08: 'GAP', 0x12: 'NACK_FRAG', 0x13: 'HB_FRAG', 0x0C: 'INFO_SRC'}
QUIET_KINDS = ('INFO_TS', 'INFO_DST')   # carried by nearly every packet: left out of the summary


def udp_of(frame):
    """(source, source_port, destination, destination_port, payload) of an Ethernet frame
    carrying IPv4 UDP, else None."""
    if len(frame) < 14 + 20 + 8 or frame[12:14] != ETHERTYPE_IPV4:
        return None
    ip = frame[14:]
    header_length = (ip[0] & 0x0F) * 4
    if ip[9] != 17 or len(ip) < header_length + 8:
        return None
    source, destination = socket.inet_ntoa(ip[12:16]), socket.inet_ntoa(ip[16:20])
    source_port, destination_port = struct.unpack('>HH', ip[header_length:header_length + 4])
    return source, source_port, destination, destination_port, ip[header_length + 8:]


def kinds_text(payload):
    """'DATA+HEARTBEAT' for an RTPS payload (INFO_TS / INFO_DST left out), '' if it isn't RTPS."""
    names = [KIND_NAMES.get(kind, hex(kind)) for kind, _, _ in submessages(payload)]
    return '+'.join(name for name in names if name not in QUIET_KINDS)


def in_domain(port, domain):
    base = discovery_port(domain)
    return base <= port < base + 100


def count_row(counts, interface, outgoing, frame, domain, peer=None):
    """Adds one captured frame to counts if it is DDS traffic of the domain (to or from peer)."""
    udp = udp_of(frame)
    if udp is None:
        return
    source, source_port, destination, destination_port, payload = udp
    if peer and peer not in (source, destination):
        return
    if not (in_domain(source_port, domain) or in_domain(destination_port, domain)):
        return
    counts[(interface, 'out' if outgoing else 'in ', f'{source}:{source_port}', f'{destination}:{destination_port}',
            kinds_text(payload))] += 1


def report(counts):
    if not counts:
        return ['no DDS packets seen']
    return [f'{interface:5s} {direction} {source:21s} -> {destination:21s} {count:4d}  {kinds}'
            for (interface, direction, source, destination, kinds), count
            in sorted(counts.items(), key=lambda item: (item[0][1], item[0][3]))]


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    domain = 42
    if '--domain' in argv:
        index = argv.index('--domain')
        domain = int(argv[index + 1])
        del argv[index:index + 2]
    seconds = float(argv[0]) if argv else 10.0
    peer = argv[1] if len(argv) > 1 else None
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(ETH_P_ALL))
    sock.settimeout(0.5)
    counts = collections.Counter()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            frame, address = sock.recvfrom(65535)
        except socket.timeout:
            continue
        count_row(counts, address[0], address[2] == PACKET_OUTGOING, frame, domain, peer)
    print('\n'.join(report(counts)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
