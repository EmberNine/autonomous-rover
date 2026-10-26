"""STATUS battery fields -> sensor_msgs/BatteryState fields. Pure: no ROS, no I/O.

The firmware's BatteryState (firmware/Core/Inc/battery_monitor.h):
  0 NOT_PRESENT  boot, or no pack (wall power: vbat is a few mV of ADC noise)
  1 OK           armed, above the 7.0 V warning
  2 LOW          armed, below the warning (motor still allowed)
  3 CUTOFF       below 6.6 V, latched until reset, motor locked out
  4 ADC_FAULT    latched until reset, motor locked out
  5 REMOVED_WHILE_DRIVING   latched until reset, motor locked out
No pack is reported as not present with voltage NaN (REP 145: not measured)
and health / status UNKNOWN, never as a 0 V dead battery. The warning (LOW)
has no BatteryState health value; it shows in /diagnostics (WARN). Charge,
capacity, current and percentage aren't measured: NaN.
"""
import math

# sensor_msgs/BatteryState constants (checked against the message in the ROS tests).
STATUS_UNKNOWN, STATUS_DISCHARGING = 0, 2
HEALTH_UNKNOWN, HEALTH_GOOD, HEALTH_DEAD, HEALTH_UNSPEC_FAILURE = 0, 1, 3, 5
TECHNOLOGY_LIPO = 3
DESIGN_CAPACITY_AH = 1.2           # the single 2S 1200 mAh pack
CELLS = 2

NOT_PRESENT, OK, LOW, CUTOFF, ADC_FAULT, REMOVED_WHILE_DRIVING = range(6)
STATE_NAMES = ('NOT_PRESENT', 'OK', 'LOW', 'CUTOFF', 'ADC_FAULT', 'REMOVED_WHILE_DRIVING')
HEALTH = {OK: HEALTH_GOOD, LOW: HEALTH_GOOD, CUTOFF: HEALTH_DEAD, ADC_FAULT: HEALTH_UNSPEC_FAILURE}


def state_name(state):
    return STATE_NAMES[state] if 0 <= state < len(STATE_NAMES) else f'STATE_{state}'


def battery_fields(status):
    """The BatteryState fields for a STATUS (a dict of message field -> value)."""
    state = status['battery_state']
    measured = state in (OK, LOW, CUTOFF)          # a pack is connected and its voltage is meaningful
    return {
        'voltage': status['vbat_mv'] / 1000.0 if measured else math.nan,
        'temperature': math.nan,
        'current': math.nan,
        'charge': math.nan,
        'capacity': math.nan,
        'design_capacity': DESIGN_CAPACITY_AH,
        'percentage': math.nan,
        'power_supply_status': STATUS_DISCHARGING if measured else STATUS_UNKNOWN,
        'power_supply_health': HEALTH.get(state, HEALTH_UNKNOWN),
        'power_supply_technology': TECHNOLOGY_LIPO,
        'present': measured,                       # ADC_FAULT: unknown, so not claimed (health says failure)
        'location': 'rover',
        'serial_number': '',
    }
