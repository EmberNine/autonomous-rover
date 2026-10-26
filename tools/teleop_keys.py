"""The laptop side of the keyboard teleop (group 4): sends keys, and a keepalive byte every
KEEPALIVE_SECONDS, over SSH to the teleop node on the Pi (rover_bridge/teleop_node.py, inside the
bridge's container), and shows what it prints. Only these bytes and the status line cross Wi-Fi;
/cmd_vel is published on the Pi, next to the bridge. Stdlib only.

    py tools\\teleop_keys.py armando@192.168.1.18          (Windows: PowerShell or cmd)
    python3 tools/teleop_keys.py armando@192.168.1.18      (Linux / WSL, in a terminal)

ssh runs as: ssh -T -e none -o IPQoS=ef -o ServerAliveInterval=2 -o ServerAliveCountMax=2 <target>
"docker exec -i rover-bridge /ros_entrypoint.sh python3 -u -m rover_bridge.teleop_node".
  -T          no tty: stdin is a clean byte pipe to the node (a remote tty would echo and line-buffer
              the keys). OpenSSH sets TCP_NODELAY either way (packet.c, ssh_packet_set_interactive),
              so Nagle doesn't hold the keepalives back; only the DSCP mark depends on the tty.
  IPQoS=ef    one value, so it applies whatever the tty: without it a no-tty session is marked
              CS1 ("lower effort", the default bulk class in 9.5 / 10.0), which Wi-Fi WMM puts in
              its background queue. AF21, the default for a tty, lands there too. EF is the video queue.
              (The Pi->laptop direction, the status line, keeps sshd's mark; the deadman doesn't
              depend on it.)
  -e none     no ~ escapes in the key stream
  ServerAlive ssh itself gives up after ~4-6 s without the Pi; the node has stopped by then (0.5 s)
              and ESTOPped (2 s) on its own.

Password login works with stdin piped: Windows OpenSSH reads the password from the console
(readpassphrase: _getwch), Linux OpenSSH from /dev/tty. So the keyboard isn't read until the
node prints its ready line: until then ssh has the console to itself (password, host key).

Keys (sent as the node's bytes): w/s speed, a/d steering (a = left), c centre, space stop,
r arm, x and Esc ESTOP, q and Ctrl+C quit (the node ESTOPs and exits; this waits for it). Other
keys are dropped here; on Windows arrows and F-keys are dropped, on Linux they arrive as Esc
sequences and are Esc (motor_console's decode_keys): ESTOP.
"""
import argparse
import os
import subprocess
import sys
import threading
import time

import motor_console
from motor_console import CTRL_C, ESC

KEEPALIVE = b'.'
KEEPALIVE_SECONDS = 0.1
POLL_SECONDS = 0.01
QUIT_WAIT_SECONDS = 3.0
READY_LINE = b'teleop: ready'
SEND = {key: key.encode() for key in 'wsadc xrq'}
REMOTE_COMMAND = 'docker exec -i {container} /ros_entrypoint.sh python3 -u -m rover_bridge.teleop_node'
SPECIAL_KEY_PREFIXES = motor_console.SPECIAL_KEY_PREFIXES


def ssh_command(target, container='rover-bridge', ssh='ssh'):
    return [ssh, '-T', '-e', 'none', '-o', 'IPQoS=ef', '-o', 'ServerAliveInterval=2',
            '-o', 'ServerAliveCountMax=2', target, REMOTE_COMMAND.format(container=container)]


def key_bytes(key):
    """One key (as motor_console's keyboards give them) -> the bytes for the node; b'' = dropped."""
    if key == ESC:
        return b'x'
    if key == CTRL_C:
        return b'q'
    return SEND.get(key.lower(), b'')


class Output:
    """The reader thread: copies the node's stdout to ours, and sets ready at READY_LINE."""

    def __init__(self, stream, write):
        self.stream = stream
        self.write = write
        self.ready = threading.Event()
        self.ended = threading.Event()
        self.seen = b''

    def run(self):
        fd = self.stream.fileno()
        while True:
            try:
                data = os.read(fd, 4096)
            except OSError:
                data = b''
            if not data:
                self.ended.set()
                self.ready.set()          # nothing more is coming: stop waiting
                return
            if not self.ready.is_set():
                self.seen = (self.seen + data)[-256:]
                if READY_LINE in self.seen:
                    self.ready.set()
            self.write(data.decode('utf-8', errors='replace'))


class Keys:
    """msvcrt on Windows, motor_console.PosixKeyboard on Linux: kbhit() / key()."""

    def __init__(self):
        try:
            import msvcrt
            self.msvcrt = msvcrt
            self.posix = None
        except ImportError:
            self.msvcrt = None
            self.posix = motor_console.PosixKeyboard(sys.stdin.fileno())

    def __enter__(self):
        if self.posix is not None:
            self.posix.__enter__()
        return self

    def __exit__(self, *exception):
        if self.posix is not None:
            return self.posix.__exit__(*exception)
        return False

    def keys(self):
        """The keys typed since the last call."""
        out = []
        if self.msvcrt is not None:
            while self.msvcrt.kbhit():
                key = self.msvcrt.getwch()
                if key in SPECIAL_KEY_PREFIXES:
                    self.msvcrt.getwch()  # arrow / F-key: the code that follows is dropped too
                    continue
                out.append(key)
        else:
            while self.posix.kbhit():
                out.append(self.posix.getwch())
        return out


def forward(process, keys, output, clock=time, sleep=time.sleep):
    """Keys and keepalives to the node until it exits; Ctrl+C (a key or KeyboardInterrupt) sends q
    and waits for the node to finish. Returns ssh's exit code."""
    last_keepalive = None
    quitting_since = None

    def send(data):
        try:
            process.stdin.write(data)
            process.stdin.flush()
            return True
        except (OSError, ValueError):
            return False                  # ssh is gone

    while process.poll() is None:
        try:
            now = clock.monotonic()
            for key in keys.keys():
                data = key_bytes(key)
                if data and not send(data):
                    break
                if data == b'q' and quitting_since is None:
                    quitting_since = now
            if last_keepalive is None or now - last_keepalive >= KEEPALIVE_SECONDS:
                send(KEEPALIVE)
                last_keepalive = now
            if quitting_since is not None and now - quitting_since > QUIT_WAIT_SECONDS:
                process.terminate()
            sleep(POLL_SECONDS)
        except KeyboardInterrupt:          # Windows: Ctrl+C reaches Python, not msvcrt
            if quitting_since is None:
                send(b'q')
                quitting_since = clock.monotonic()
    return process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Keyboard teleop over SSH (docs/ros-bridge.md, Teleop).')
    parser.add_argument('target', help='user@host of the Pi, e.g. armando@192.168.1.18')
    parser.add_argument('--container', default='rover-bridge')
    parser.add_argument('--ssh', default='ssh', help='the ssh program (default: ssh on PATH)')
    args = parser.parse_args(argv)
    command = ssh_command(args.target, args.container, args.ssh)
    print('starting: ' + ' '.join(command), flush=True)
    # Windows: ssh in its own process group, so Ctrl+C reaches this script (which sends q) and not
    # ssh (which would cut the link: the node would ESTOP on end of input instead of on q).
    flags = getattr(subprocess, 'CREATE_NEW_PROCESS_GROUP', 0)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0,
                               creationflags=flags)

    def write(text):
        sys.stdout.write(text)
        sys.stdout.flush()
    output = Output(process.stdout, write)
    threading.Thread(target=output.run, daemon=True).start()
    try:
        while not output.ready.wait(0.1):  # ssh's password / host-key prompt has the console
            if process.poll() is not None:
                break
    except KeyboardInterrupt:
        process.terminate()
        return process.wait()
    if output.ended.is_set() or process.poll() is not None:
        code = process.wait()
        print(f'\nssh exited ({code}) before the teleop was ready')
        return code or 1
    with motor_console.QuickEditOff(motor_console.load_kernel32(), lambda text: None), Keys() as keys:
        code = forward(process, keys, output)
    output.ended.wait(1.0)
    print(f'\nssh exited ({code}); the teleop on the Pi has stopped publishing')
    return code


if __name__ == '__main__':
    sys.exit(main())
