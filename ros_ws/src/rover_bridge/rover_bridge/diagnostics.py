"""What the bridge reports on /diagnostics. Pure: no ROS, no I/O.

Diagnostics.build(snapshot) returns one status per part, each a dict:
name, level (OK / WARN / ERROR / STALE as in diagnostic_msgs), message,
values [(key, text)]. The node turns them into a DiagnosticArray (1 Hz).

  rover_bridge: motor     motor state, latch / fault reason by name, IMU, build and reset flags
  rover_bridge: battery   vbat, the firmware's battery state, VDDA
  rover_bridge: link      this side's counts (frames, bad, gaps, stale, read gaps, queue drops)
                          and the firmware's own receive counters from STATUS
  rover_bridge: clock     the board-time fit: drift, offset, residual, latency
  rover_bridge: operator  driving: state, heartbeat, the speed asked for (/cmd_vel) and the one
                          being sent, heartbeat stops and /cmd_vel gaps, the last event and its age

Levels: ERROR for what makes driving wrong (motor FAULT, a test build, no
IMU, battery cutoff / ADC fault / removed), WARN for what got worse since the
previous report (new bad pieces or gaps, a read gap over READ_GAP_WARN, queue
drops) or a fit that isn't there yet or doesn't fit, STALE when no STATE /
STATUS came for STALE_SECONDS.
"""
from rover_bridge import _tools  # noqa: F401
import rover_link  # noqa: E402
from motor_console import BATTERY_STATE_MARKERS, MOTOR_FAULT_NAMES, reset_cause_text  # noqa: E402,F401

from rover_bridge import battery, power_state  # noqa: E402

OK, WARN, ERROR, STALE = 0, 1, 2, 3          # diagnostic_msgs/DiagnosticStatus levels
STALE_SECONDS = 1.0
READ_GAP_WARN = 0.5                           # s; the buffers before the port hold ~0.9 s
RESIDUAL_WARN = 0.002                         # s
DRIFT_RANGE_PPM = (-1000.0, 10000.0)          # outside: the fit (or a clock) is off
MOTOR_STATES = ('OFF', 'RUNNING', 'LATCHED', 'FAULT')
FIRMWARE_LINK_COUNTERS = ('rx_ok', 'rx_crc_errors', 'rx_frame_errors', 'rx_rejected', 'rx_seq_gaps',
                          'rx_duplicates', 'rx_uart_errors', 'rx_ring_drops', 'rx_text_ignored', 'tx_dropped',
                          'overruns', 'resyncs', 'watchdog_trips', 'latches')
WORSE_COUNTS = ('bad', 'seq_gaps', 't_ms_gaps', 'dropped')
SAFETY_STOP_AGE = 0.3                         # s: safety.HEARTBEAT_STOP


def _name(names, code):
    return names[code] if 0 <= code < len(names) else f'{code}?'


def _ms(seconds):
    return 'n/a' if seconds is None else f'{seconds * 1000:.2f}'


def status(name, level, message, values):
    return {'name': name, 'level': level, 'message': message, 'values': [(key, str(value)) for key, value in values]}


class Diagnostics:
    """Keeps the previous report's counts, so a level can say "got worse since"."""

    def __init__(self):
        self.previous = None

    def build(self, snapshot):
        """snapshot: status (STATUS fields or None), hello_ack (fields or None), state_age and
        status_age (s since the last one, None if never), counts (dict: frames, bad, seq_gaps,
        t_ms_gaps, dropped, hello_sent), stale_text, read_gap_period, read_gap_session (s or None),
        clock (a ClockFit)."""
        reports = [self._motor(snapshot), self._battery(snapshot), self._link(snapshot), self._clock(snapshot),
                   self._operator(snapshot)]
        self.previous = dict(snapshot['counts'])
        self.previous_tx_dropped = snapshot['status']['tx_dropped'] if snapshot['status'] is not None else None
        safety = snapshot.get('safety')
        self.previous_safety = dict(safety['counts']) if safety else None
        return reports

    previous_safety = None
    previous_tx_dropped = None

    def _operator(self, snapshot):
        """The driving side (safety.Safety.snapshot): state, heartbeat, the last command."""
        safety = snapshot.get('safety')
        if safety is None:
            return status('rover_bridge: operator', OK, 'no heartbeat yet (read-only bridge)', [('heartbeat age', 'n/a')])
        age = safety['heartbeat_age']
        age_event = safety['last_event_age']
        counts = safety['counts']
        values = [('state', safety['state']), ('heartbeat age ms', _ms(age)),
                  ('command cps (from /cmd_vel)', safety['command_cps']),
                  ('drive cps (sent now)', 'n/a' if safety['drive_cps'] is None else safety['drive_cps']),
                  ('command steer % (from /cmd_vel)', safety['command_steer']),
                  ('drive steer % (sent now)', 'n/a' if safety['drive_steer'] is None else safety['drive_steer']),
                  ('last event', safety['last_event']),
                  ('last event age s', 'n/a' if age_event is None else f'{age_event:.1f}')]
        values += [(key, value) for key, value in counts.items()]
        values += [('heartbeat stops since arm', safety['stops_since_arm']),
                   ('cmd_vel gap max ms (since arm)', _ms(safety['gap_max'])),
                   ('cmd_vel last stop gap ms (since arm)', _ms(safety['last_stop_gap'])),
                   ('cmd_vel gaps >= 150 ms (since arm)', safety['slow_gaps_since_arm'])]
        if safety['shutting_down']:
            return status('rover_bridge: operator', WARN, 'shutting down: ESTOP sent', values)
        if safety['state'] != 'DISARMED' and age is not None and age > SAFETY_STOP_AGE:
            return status('rover_bridge: operator', WARN, f'no /cmd_vel for {age * 1000:.0f} ms: DRIVE 0', values)
        new = [key for key in ('below minimum', 'clamped', 'invalid', 'steering clamped', 'steering invalid')
               if self.previous_safety is not None and counts[key] > self.previous_safety.get(key, 0)]
        if new:
            text = {'below minimum': 'commands below 1000 cps sent as 0', 'clamped': 'commands clamped at 5000 cps',
                    'invalid': 'non-finite commands sent as 0', 'steering clamped': 'steering clamped at full lock',
                    'steering invalid': 'non-finite commands steered to centre'}
            return status('rover_bridge: operator', WARN, '; '.join(text[key] for key in new), values)
        when = 'at start' if age_event is None else f'{age_event:.1f} s ago'
        return status('rover_bridge: operator', OK, f"{safety['state']}: {safety['last_event']} ({when})", values)

    def _motor(self, snapshot):
        status_fields, ack = snapshot['status'], snapshot['hello_ack']
        if status_fields is None or snapshot['status_age'] is None or snapshot['status_age'] > STALE_SECONDS:
            return status('rover_bridge: motor', STALE, 'no STATUS from the board', [])
        state = status_fields['motor_state']
        text = _name(MOTOR_STATES, state)
        latch = _name(rover_link.LATCH_REASON_NAMES, status_fields['latch_reason'])
        fault = _name(MOTOR_FAULT_NAMES, status_fields['fault_reason'])
        message = f'{text} ({latch})' if state == 2 else f'{text} ({fault})' if state == 3 else text
        build_flags = ack['build_flags'] if ack else None
        speed = snapshot.get('speed_cps')
        values = [('state', text), ('latch reason', latch), ('fault reason', fault),
                  # STATUS echoes the last DRIVE's speed as received (link_rx.c); the controller
                  # runs on it in 50-cps units (LinkProtocol_CpsToUnits): 2542 -> 2550.
                  ('setpoint cps (board echo of last DRIVE)', status_fields['setpoint_cps']),
                  ('setpoint cps (board, 50-cps units)',
                   rover_link.cps_to_units(status_fields['setpoint_cps']) * rover_link.CPS_PER_UNIT),
                  ('speed cps (estimate)', 'n/a' if speed is None else speed),
                  ('imu ok', bool(status_fields['imu_ok'])),
                  ('build flags', 'n/a' if ack is None else (rover_link.build_flag_text(build_flags) or 'none (normal build)')),
                  ('reset flags', f"{status_fields['reset_flags']} ({reset_cause_text(status_fields['reset_flags'])})")]
        level = OK
        if state == 3 or not status_fields['imu_ok'] or build_flags:
            level = ERROR
            if build_flags:
                message += '; TEST BUILD: never drive the truck with it'
            if not status_fields['imu_ok']:
                message += '; IMU failed at boot'
        return status('rover_bridge: motor', level, message, values)

    def _battery(self, snapshot):
        status_fields = snapshot['status']
        if status_fields is None or snapshot['status_age'] is None or snapshot['status_age'] > STALE_SECONDS:
            return status('rover_bridge: battery', STALE, 'no STATUS from the board', [])
        state = status_fields['battery_state']
        values = [('state', battery.state_name(state)), ('vbat mV', status_fields['vbat_mv']),
                  ('vdda mV', status_fields['vdda_mv'])]
        if 'power_watch' in snapshot:
            watch = snapshot['power_watch']
            values.append(('Pi shutdown watch', power_state.watch_text(watch)))
            if watch is not None and watch['state'] in ('low', 'shutdown'):
                report = self._battery_level(state, status_fields, values)
                level = max(report['level'], WARN)
                return status('rover_bridge: battery', level, f"{report['message']}; {power_state.watch_text(watch)}", values)
        return self._battery_level(state, status_fields, values)

    def _battery_level(self, state, status_fields, values):
        if state == battery.NOT_PRESENT:
            return status('rover_bridge: battery', OK, 'no pack connected (e.g. on wall power)', values)
        if state == battery.OK:
            return status('rover_bridge: battery', OK, f"{status_fields['vbat_mv'] / 1000:.2f} V", values)
        if state == battery.LOW:
            return status('rover_bridge: battery', WARN,
                          f"LOW: {status_fields['vbat_mv'] / 1000:.2f} V, below the 7.0 V warning", values)
        return status('rover_bridge: battery', ERROR, f'{battery.state_name(state)}: motor locked out until reset',
                      values)

    def _link(self, snapshot):
        counts = snapshot['counts']
        values = [('frames', counts['frames']), ('bad', counts['bad']), ('seq gaps', counts['seq_gaps']),
                  ('t_ms gaps', counts['t_ms_gaps']), ('stale before HELLO_ACK', snapshot['stale_text']),
                  ('queue drops', counts['dropped']), ('hello sent', counts['hello_sent']),
                  ('read gap max ms (period)', _ms(snapshot['read_gap_period'])),
                  ('read gap max ms (session)', _ms(snapshot['read_gap_session'])),
                  ('STATE age ms', _ms(snapshot['state_age']))]
        if snapshot['status'] is not None:
            values += [(f'firmware {key}', snapshot['status'][key]) for key in FIRMWARE_LINK_COUNTERS]
        if snapshot['state_age'] is None or snapshot['state_age'] > STALE_SECONDS:
            what = 'no HELLO_ACK yet' if snapshot['hello_ack'] is None else 'no STATE'
            return status('rover_bridge: link', STALE, what, values)
        worse = [key for key in WORSE_COUNTS
                 if self.previous is not None and counts[key] > self.previous.get(key, 0)]
        problems = [f'new {key.replace("_", " ")}: +{counts[key] - self.previous.get(key, 0)}' for key in worse]
        if 'seq_gaps' in worse and snapshot['status'] is not None and self.previous_tx_dropped is not None:
            # A frame the board never sent (its TX queue was full) is a seq gap here too.
            new_gaps = counts['seq_gaps'] - self.previous.get('seq_gaps', 0)
            board = snapshot['status']['tx_dropped'] - self.previous_tx_dropped
            if board == new_gaps:
                problems[worse.index('seq_gaps')] += f' (dropped by the board: tx_dropped +{board})'
        if snapshot['read_gap_period'] is not None and snapshot['read_gap_period'] > READ_GAP_WARN:
            problems.append(f"read gap {snapshot['read_gap_period'] * 1000:.0f} ms")
        if problems:
            return status('rover_bridge: link', WARN, '; '.join(problems), values)
        return status('rover_bridge: link', OK, f"{counts['frames']} frames, no losses", values)

    def _clock(self, snapshot):
        fit = snapshot['clock']
        drift, residual = fit.drift_ppm(), fit.residual()
        values = [('drift ppm (board fast +)', 'n/a' if drift is None else f'{drift:.0f}'),
                  ('offset ms (Pi - board)', _ms(fit.offset())), ('residual ms', _ms(residual)),
                  ('latency ms', _ms(fit.latency())), ('window s', f'{fit.span():.1f}'),
                  ('bins rejected (late)', fit.rejected), ('stamps held back (monotonic)', fit.clamps),
                  ('fit restarts (board resets)', fit.resets), ('host clock steps', fit.host_steps)]
        if drift is None:
            return status('rover_bridge: clock', WARN, 'fitting (needs ~3 s of STATE)', values)
        if not DRIFT_RANGE_PPM[0] <= drift <= DRIFT_RANGE_PPM[1] or residual > RESIDUAL_WARN:
            return status('rover_bridge: clock', WARN,
                          f'fit looks wrong: drift {drift:.0f} ppm, residual {residual * 1000:.2f} ms', values)
        return status('rover_bridge: clock', OK, f'board {drift:+.0f} ppm, residual {residual * 1000:.2f} ms', values)
