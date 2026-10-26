#!/usr/bin/env python3
"""Log the rover firmware's telemetry from a serial port, byte-for-byte.

Writes two files:
  <output>            the raw bytes exactly as received (nothing added, nothing removed)
  <output>.times.csv  one row per received line: host time, line number, byte offset, ok/bad

and keeps a running count of lines that pass a strict format check. Close any
other program that has the port open (e.g. the VS Code Serial Monitor) first:
only one program can hold a COM port at a time.

Usage:  py tools\\serial_logger.py COM5 logs\\gyro_static.log [--baud 115200]
"""
import argparse
import re
import sys
import time

# Every field of a telemetry line, in the order the firmware prints them.
EXPECTED_FIELDS = (
    't_ms', 'raw', 'count', 'd100', 'MD', 'ML', 'MH', 'AGC', 'mag_min', 'mag_max',
    'i2c_err', 'overrun', 'resync', 'skipped', 'i2c_us_max', 'loop_us_max',
    'imu_ok', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp',
    'gsum_x', 'gsum_y', 'gsum_z', 'n_imu', 'imu_err', 'spi_us_max',
)
# One exact pattern: "t_ms=<int> raw=<int> ... spi_us_max=<int>\r\n", single spaces, nothing else.
STRICT_LINE = re.compile(
    rb'^' + rb' '.join(name.encode() + rb'=-?\d+' for name in EXPECTED_FIELDS) + rb'\r\n$'
)
BOOT_LINE = re.compile(rb'^imu_init=\w+ who=0x[0-9A-F]{2}\r\n$')
REPORT_PERIOD_MS = 100      # firmware sends one line every 100 ms
STATUS_EVERY_SECONDS = 2.0  # how often the running totals are printed
MAX_BAD_LINES_SHOWN = 5     # print the first few bad lines in full, to see what's wrong


def classify(line):
    """Return 'good', 'boot' or 'bad' for one complete line (ending in b'\\n')."""
    if STRICT_LINE.match(line):
        return 'good'
    if BOOT_LINE.match(line):
        return 'boot'
    return 'bad'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', help='serial port, e.g. COM5')
    parser.add_argument('output', help='file for the raw bytes; timestamps go to <output>.times.csv')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    import serial  # imported here so the line checker above can be used without pyserial

    # timeout=0.1 makes read() return at least every 100 ms, so Ctrl+C is noticed promptly.
    port = serial.Serial(args.port, args.baud, timeout=0.1)
    counts = {'good': 0, 'bad': 0, 'boot': 0}
    gaps = 0               # good lines whose t_ms didn't follow the previous one by exactly 100
    previous_t_ms = None
    partial_line = b''
    byte_offset = 0        # where in the raw file the current line starts
    line_number = 0
    discarded_first = False
    next_status_time = time.monotonic() + STATUS_EVERY_SECONDS

    with open(args.output, 'wb') as raw_file, \
         open(args.output + '.times.csv', 'w', encoding='ascii', newline='') as times_file:
        times_file.write('host_time_unix,line,byte_offset,result\n')
        try:
            while True:
                chunk = port.read(port.in_waiting or 1)
                received_at = time.time()  # all lines finished in this chunk get the same host time
                if chunk:
                    raw_file.write(chunk)  # unchanged, before any parsing
                partial_line += chunk

                while b'\n' in partial_line:
                    line, partial_line = partial_line.split(b'\n', 1)
                    line += b'\n'
                    line_number += 1
                    if not discarded_first:
                        # Opening the port mid-line gives a truncated first line; don't count it.
                        discarded_first = True
                        result = 'first_partial' if not line.startswith((b't_ms=', b'imu_init=')) else classify(line)
                    else:
                        result = classify(line)

                    if result in counts:
                        counts[result] += 1
                    if result == 'bad' and counts['bad'] <= MAX_BAD_LINES_SHOWN:
                        print(f'\nBAD line {line_number}: {line!r}')
                    if result == 'good':
                        t_ms = int(line[5:line.index(b' ')])
                        if previous_t_ms is not None and t_ms - previous_t_ms != REPORT_PERIOD_MS:
                            gaps += 1
                        previous_t_ms = t_ms
                    if result == 'boot':
                        previous_t_ms = None  # the board restarted; t_ms starts over

                    times_file.write(f'{received_at:.6f},{line_number},{byte_offset},{result}\n')
                    byte_offset += len(line)

                if time.monotonic() >= next_status_time:
                    raw_file.flush()
                    times_file.flush()
                    total = counts['good'] + counts['bad']
                    bad_percent = 100.0 * counts['bad'] / total if total else 0.0
                    print(f"\rgood={counts['good']} bad={counts['bad']} ({bad_percent:.2f}%) "
                          f"boot={counts['boot']} t_ms_gaps={gaps}   ", end='', flush=True)
                    next_status_time += STATUS_EVERY_SECONDS
        except KeyboardInterrupt:
            pass
        finally:
            port.close()

    total = counts['good'] + counts['bad']
    print(f"\nStopped. {total} lines: good={counts['good']} bad={counts['bad']} "
          f"boot={counts['boot']} t_ms_gaps={gaps}")
    return 0 if counts['bad'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
