#!/usr/bin/env python3
"""Decode a raw binary-mode log (Pi link) into one CSV per frame type.

Reads the raw file written by serial_logger.py --binary or motor_console.py
--binary (exactly the bytes received), splits it into frames with the same
code (serial_logger.BinaryLogger, tools/rover_link.py), and writes:

  <prefix>.state.csv      piece, seq, then every STATE field (100 Hz)
  <prefix>.status.csv     piece, seq, then every STATUS field (10 Hz)
  <prefix>.hello_ack.csv  piece, seq, then every HELLO_ACK field

<prefix> is the log's path unless --out is given. Prints a summary: frames
per type, bad pieces by reason, sequence gaps per type, STATE t_ms steps
other than 10 ms, boot lines and text lines (a board in text mode). Text
between frames (a boot line, v5 lines) is recognised and not counted as bad.
Pieces before the first HELLO_ACK are stale (serial_logger.BinaryLogger:
the ST-LINK bridge's leftover from before the port was opened): counted on
their own ("stale before HELLO_ACK"), not as frames, bad pieces or gaps, and
not in the CSV files.

Exit code 0 if there was no bad piece and no gap after the first HELLO_ACK,
else 1; also 1 if the log has no HELLO_ACK at all. Stale data alone: 0.

Usage:  py tools\\link_dump.py C:\\dev\\rover-logs\\binary_soak.log [--out C:\\dev\\rover-logs\\soak]
"""
import argparse
import csv
import sys

import rover_link
from serial_logger import BinaryLogger

CSV_TYPES = {'STATE': ('state', rover_link.STATE_FIELDS), 'STATUS': ('status', rover_link.STATUS_FIELDS),
             'HELLO_ACK': ('hello_ack', rover_link.HELLO_ACK_FIELDS)}


def dump(data):
    """(logger, rows): the BinaryLogger after the whole log, and per CSV type
    the rows [piece, seq, *fields] in order."""
    logger = BinaryLogger(None, None, print_function=lambda text: None)
    rows = {name: [] for name in CSV_TYPES}
    for event in logger.feed(data, 0.0):
        if event[0] == 'frame' and event[1] in rows and event[3] is not None:
            _, name, seq, fields, piece = event   # the frame's own piece number (as in <log>.times.csv)
            rows[name].append([piece, seq] + [fields[key] for key in CSV_TYPES[name][1]])
    return logger, rows


def write_csv_files(prefix, rows):
    """Writes the three CSV files; returns their paths."""
    paths = []
    for name, (suffix, fields) in CSV_TYPES.items():
        path = f'{prefix}.{suffix}.csv'
        with open(path, 'w', encoding='ascii', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['piece', 'seq', *fields])
            writer.writerows(rows[name])
        paths.append(path)
    return paths


def exit_code(logger):
    """1 for a bad piece or a gap after the first HELLO_ACK, or for no HELLO_ACK at all; stale data alone: 0."""
    clean = logger.counts['bad'] == 0 and not logger.seq_gaps and logger.t_ms_gaps == 0
    return 0 if clean and not logger.awaiting_hello_ack else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', help='raw binary-mode log')
    parser.add_argument('--out', help='prefix for the CSV files (default: the log path)')
    args = parser.parse_args(argv)
    with open(args.log, 'rb') as file:
        data = file.read()
    logger, rows = dump(data)
    for path in write_csv_files(args.out or args.log, rows):
        print(f'wrote {path}')
    print(logger.summary_text().replace('Stopped. ', ''))
    return exit_code(logger)


if __name__ == '__main__':
    sys.exit(main())
