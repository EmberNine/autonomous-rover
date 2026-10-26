"""Tests for teleop_keys.py (the laptop side of the teleop): the ssh command, the key mapping, the
keepalive cadence, Ctrl+C -> q, and waiting for the node's ready line. No ssh, no ROS."""
import os
import threading

import pytest

import teleop_keys
from motor_console import CTRL_C, ESC


def test_the_ssh_command_has_no_tty_ef_marking_and_no_escapes():
    command = teleop_keys.ssh_command('armando@192.168.1.18')
    assert command[:2] == ['ssh', '-T']
    pairs = list(zip(command, command[1:]))
    assert ('-o', 'IPQoS=ef') in pairs and ('-e', 'none') in pairs
    assert ('-o', 'ServerAliveInterval=2') in pairs and ('-o', 'ServerAliveCountMax=2') in pairs
    assert command[-2] == 'armando@192.168.1.18'
    assert command[-1] == 'docker exec -i rover-bridge /ros_entrypoint.sh python3 -u -m rover_bridge.teleop_node'
    assert '-t' not in command[-1].split() and '-it' not in command[-1].split()   # a pipe, not a tty


def test_keys_map_to_the_nodes_bytes():
    assert [teleop_keys.key_bytes(key) for key in 'wsadc xrq'] == [key.encode() for key in 'wsadc xrq']
    assert teleop_keys.key_bytes('W') == b'w' and teleop_keys.key_bytes('A') == b'a'
    assert teleop_keys.key_bytes(ESC) == b'x'                    # Esc: ESTOP
    assert teleop_keys.key_bytes(CTRL_C) == b'q'                 # Ctrl+C: quit
    assert teleop_keys.key_bytes('.') == b''                     # the keepalive is never a key
    assert teleop_keys.key_bytes('z') == b'' and teleop_keys.key_bytes('\r') == b''


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Stdin:
    def __init__(self):
        self.sent = []

    def write(self, data):
        self.sent.append(data)

    def flush(self):
        pass


class Process:
    """ssh: running until exit_at (fake time), or until terminate()."""

    def __init__(self, clock, exit_at):
        self.clock, self.exit_at = clock, exit_at
        self.stdin = Stdin()
        self.terminated = False

    def poll(self):
        return 0 if self.terminated or self.clock.now >= self.exit_at else None

    def wait(self):
        return 0

    def terminate(self):
        self.terminated = True


class Keys:
    def __init__(self, clock, script):
        self.clock, self.script = clock, sorted(script, key=lambda item: item[0])   # (at, key or KeyboardInterrupt)

    def keys(self):
        out = []
        while self.script and self.script[0][0] <= self.clock.now:
            _at, key = self.script.pop(0)
            if key is KeyboardInterrupt:
                raise KeyboardInterrupt
            out.append(key)
        return out


def run(script, exit_after):
    clock = Clock()
    process = Process(clock, clock.now + exit_after)
    keys = Keys(clock, [(clock.now + at, key) for at, key in script])
    code = teleop_keys.forward(process, keys, None, clock=clock, sleep=clock.sleep)
    return code, process


def test_a_keepalive_every_100_ms_and_the_keys_in_order():
    code, process = run([(0.25, 'w'), (0.25, 'a'), (0.5, 'W')], exit_after=1.0)
    assert code == 0
    keepalives = process.stdin.sent.count(teleop_keys.KEEPALIVE)
    assert 10 <= keepalives <= 11                                # 1 s at 10 Hz
    assert [data for data in process.stdin.sent if data != teleop_keys.KEEPALIVE] == [b'w', b'a', b'w']


def test_ctrl_c_sends_q_once_and_waits_for_the_node():
    code, process = run([(0.3, KeyboardInterrupt), (0.4, KeyboardInterrupt)], exit_after=1.0)
    assert [data for data in process.stdin.sent if data != teleop_keys.KEEPALIVE] == [b'q']
    assert not process.terminated                                # the node finished by itself (at 1 s)


def test_a_node_that_doesnt_finish_after_q_is_terminated():
    _code, process = run([(0.3, 'q')], exit_after=60.0)
    assert process.terminated


def test_ready_is_seen_across_reads_and_everything_is_shown():
    read_end, write_end = os.pipe()
    shown = []
    output = teleop_keys.Output(os.fdopen(read_end, 'rb', buffering=0), shown.append)
    thread = threading.Thread(target=output.run, daemon=True)
    thread.start()
    os.write(write_end, b'[1.0] wheelbase 0.133 m\nteleop: re')
    assert not output.ready.wait(0.2)
    os.write(write_end, b'ady\n')
    assert output.ready.wait(2.0) and not output.ended.is_set()
    os.close(write_end)
    assert output.ended.wait(2.0)
    assert ''.join(shown) == '[1.0] wheelbase 0.133 m\nteleop: ready\n'


def test_ssh_ending_before_ready_also_ends_the_wait():
    read_end, write_end = os.pipe()
    output = teleop_keys.Output(os.fdopen(read_end, 'rb', buffering=0), lambda text: None)
    threading.Thread(target=output.run, daemon=True).start()
    os.write(write_end, b'Permission denied (publickey,password).\n')
    os.close(write_end)
    assert output.ready.wait(2.0) and output.ended.wait(2.0)


@pytest.mark.parametrize('marker', [teleop_keys.READY_LINE])
def test_the_ready_line_is_the_nodes(marker):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros_ws' / 'src' / 'rover_bridge'))
    text = (Path(__file__).resolve().parents[1] / 'ros_ws/src/rover_bridge/rover_bridge/teleop_node.py').read_text()
    assert f"READY_LINE = '{marker.decode()}'" in text
