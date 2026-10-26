"""rover_bridge.link: HELLO only, the stale rule, the queue, a lost port. No ROS (CI included)."""
import time

import bridge_fakes
from bridge_fakes import StepClock, StepPort, hello_ack_frame, state_frame, status_frame, written_types
from rover_bridge.link import EVENT_QUEUE_LENGTH, Link, usb_serial_tty
import rover_link
import serial_logger


def steps(link, seconds):
    """Both loops, step by step, for this long of clock time (each read takes 20 ms)."""
    end = link.clock.now + seconds
    while link.clock.now < end:
        link.hello_once()
        link.read_once()


def started_link():
    clock = StepClock()
    port = StepPort(clock)
    return Link(port, clock=clock), port, clock


def test_only_hello_is_ever_written_first_after_the_start_delay():
    link, port, clock = started_link()
    start = clock.now
    steps(link, 0.09)
    assert port.writes == []                                  # HELLO_START_DELAY_SECONDS: not yet
    steps(link, 0.05)
    assert written_types(port.writes) == [rover_link.HELLO]
    steps(link, 1.0)                                          # unanswered: again every 0.5 s
    assert written_types(port.writes) == [rover_link.HELLO] * 3
    port.give(hello_ack_frame() + b''.join(state_frame(n, 10 * n) for n in range(50)))
    steps(link, 3.0)                                          # answered: no more
    assert written_types(port.writes) == [rover_link.HELLO] * 3
    assert clock.now - start > 4.0


def test_leftover_before_the_hello_is_stale_and_never_queued():
    link, port, clock = started_link()
    port.give(b''.join(state_frame(n, 900 + 10 * n) for n in range(5)) + hello_ack_frame(seq=5))   # the bridge's leftover
    steps(link, 0.06)
    assert port.writes == [] and link.take_events() == []
    steps(link, 0.1)                                          # the HELLO goes out
    port.give(hello_ack_frame() + state_frame(40, 5000) + status_frame(4) + state_frame(41, 5010))
    steps(link, 0.1)
    events = link.take_events()
    assert [name for _, name, _, _, _ in events] == ['HELLO_ACK', 'STATE', 'STATUS', 'STATE']
    assert all(read_at == events[0][4] for *_, read_at in events)   # one read, one monotonic time
    assert events[1][3]['t_ms'] == 5000
    assert link.logger.stale_counts == {'frames': 6, 'bad': 0}
    assert any(note.startswith('HELLO_ACK: protocol 1, normal build') for note in link.take_notes())


def test_a_board_reset_brings_the_hello_again():
    link, port, clock = started_link()
    steps(link, 0.2)
    port.give(hello_ack_frame() + state_frame(0, 10))
    steps(link, 0.1)
    assert written_types(port.writes) == [rover_link.HELLO]
    port.give(bridge_fakes.BOOT_LINE)
    steps(link, serial_logger.HELLO_AFTER_BOOT_SECONDS + 0.1)
    assert written_types(port.writes) == [rover_link.HELLO] * 2
    assert any(note.startswith('board reset') for note in link.take_notes())


def test_read_gap_and_dropped_events_are_counted():
    link, port, clock = started_link()
    steps(link, 0.2)
    port.give(hello_ack_frame())
    steps(link, 0.04)
    link.take_read_gap_max()
    clock.now += 1.5                                          # the reader was held up
    port.give(b''.join(state_frame(n, 10 * n) for n in range(EVENT_QUEUE_LENGTH + 30)))
    while port.pending:
        link.read_once()
    link.read_once()                                          # a normal 20 ms read after it: the max holds
    assert 1.5 < link.take_read_gap_max() < 1.6 and link.take_read_gap_max() == 0.0
    assert link.dropped_events == 31                          # HELLO_ACK + 30 STATE: the oldest go
    assert len(link.take_events()) == EVENT_QUEUE_LENGTH
    assert 'dropped=31' in link.stats_text()


def test_a_lost_port_sets_failed_and_stop_closes():
    port = StepPort(StepClock())
    link = Link(port, clock=time)

    def slow_read(size, read=port.read):
        time.sleep(0.005)
        return read(size)

    port.read = slow_read
    link.start()
    time.sleep(0.05)
    assert not link.failed.is_set()
    port.error = OSError(5, 'device reports readiness to read but returned no data')
    assert link.failed.wait(1.0) and isinstance(link.failure, OSError)
    link.stop()
    assert port.closed


NUCLEO_SERIAL = '0673FF535451897067112541'


def fake_sysfs(root, devices):
    """root/class/tty/<tty>/device -> root/devices/<usb>/<usb>:1.2, with <usb>/serial; devices: {tty: serial}."""
    for number, (tty, serial) in enumerate(devices.items()):
        usb = root / 'devices' / f'1-{number + 1}'
        interface = usb / f'1-{number + 1}:1.2'
        interface.mkdir(parents=True)
        if serial is not None:
            (usb / 'serial').write_text(serial + '\n')
        (root / 'class' / 'tty' / tty).mkdir(parents=True)
        (root / 'class' / 'tty' / tty / 'device').symlink_to(interface)
    (root / 'class' / 'tty' / 'ttyS0').mkdir(parents=True, exist_ok=True)       # no device link
    return str(root / 'class' / 'tty')


def test_usb_serial_tty_finds_the_board_by_serial(tmp_path):
    sysfs = fake_sysfs(tmp_path, {'ttyACM0': 'X' + NUCLEO_SERIAL, 'ttyACM1': NUCLEO_SERIAL, 'ttyUSB0': None})
    assert usb_serial_tty(NUCLEO_SERIAL, sysfs) == 'ttyACM1'           # the whole serial, not a part of it
    assert usb_serial_tty('NOT_THERE', sysfs) is None
    assert usb_serial_tty(NUCLEO_SERIAL, str(tmp_path / 'missing')) is None


def test_stop_is_not_a_failure():
    port = StepPort(StepClock())
    link = Link(port, clock=time)

    def read(size):
        time.sleep(0.005)
        if link.stopping.is_set():                            # a read cut short by the shutdown
            raise OSError(9, 'Bad file descriptor')
        return b''

    port.read = read
    link.start()
    time.sleep(0.05)
    link.stop()
    assert port.closed and not link.failed.is_set() and link.failure is None


def test_the_tx_step_encodes_drive_with_the_centre_explicitly_and_estop_on_one_sequence():
    link, port, clock = started_link()
    actions = [[('DRIVE', 2542, 0, True)], [('DRIVE', -1200, 0, False)], [('ESTOP',)], []]
    link.tx_source = lambda now: actions.pop(0) if actions else []
    steps(link, 0.2)                                          # the HELLO (after 0.1 s) and the four steps
    decoder = rover_link.FrameDecoder()
    decoder.feed(b'\x00')
    frames = [frame for data in port.writes for status, frame, _ in decoder.feed(data)]
    drives = [rover_link.unpack_drive(frame.payload) for frame in frames if frame.type == rover_link.DRIVE]
    assert drives == [{'speed_cps': 2542, 'steer_percent': 0, 'steer_valid': True, 'arm': True},
                      {'speed_cps': -1200, 'steer_percent': 0, 'steer_valid': True, 'arm': False}]
    assert [frame.type for frame in frames].count(rover_link.ESTOP) == 1
    assert [frame.seq for frame in frames] == list(range(len(frames)))   # one sequence: HELLO, DRIVE, ESTOP
    assert link.sent == {'DRIVE': 2, 'ESTOP': 1, 'HELLO': link.sent['HELLO']} and link.sent['HELLO'] >= 1
