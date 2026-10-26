"""The bridge's side of the Pi's low-battery shutdown (group 5; the watcher is tools/battery_watch.py,
a host systemd service). Pure apart from the file writes.

Once a second the bridge writes <state_dir>/battery.json (the host's /run/rover, mounted at the
same path): boot_id, status_monotonic (when the STATUS was received, CLOCK_MONOTONIC: the
container and the host share it, so a wall-clock step can't make old data look fresh),
battery_state, vbat_mv, motor_state, and wall (the Pi's time, for logs). Written as a temporary
file and renamed, so the watcher never reads half of one. No STATUS yet: no file (the watcher
reads that as stale).

It reads back <state_dir>/watch.json, the watcher's state, for /diagnostics (the battery status)
and so the teleop's status line: fresh when from this boot and at most WATCH_STALE_SECONDS old.
"""
import json
import os

BOOT_ID_FILE = '/proc/sys/kernel/random/boot_id'
WATCH_STALE_SECONDS = 5.0


def read_boot_id(path=BOOT_ID_FILE):
    try:
        with open(path, encoding='ascii') as file:
            return file.read().strip()
    except OSError:
        return ''


def battery_state(status, status_at, boot_id, wall):
    """The battery.json payload for the last STATUS (fields) and when it was received."""
    return {'boot_id': boot_id, 'status_monotonic': status_at, 'battery_state': status['battery_state'],
            'vbat_mv': status['vbat_mv'], 'motor_state': status['motor_state'], 'wall': wall}


def write_atomic(path, data):
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as file:
        json.dump(data, file)
    os.replace(temporary, path)


def read_watch(path, now, boot_id):
    """The watcher's state if fresh, else None (no watcher, or it stopped)."""
    try:
        with open(path, encoding='utf-8') as file:
            watch = json.load(file)
        if watch['boot_id'] == boot_id and 0.0 <= now - watch['monotonic'] <= WATCH_STALE_SECONDS:
            return watch
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return None


def watch_text(watch):
    """For /diagnostics: the watcher's state in a few words."""
    if watch is None:
        return 'not running'
    if watch['state'] == 'low':
        return f"Pi SHUTDOWN in {watch['shutdown_in_s']:.0f} s ({watch['action']})"
    if watch['state'] == 'shutdown':
        return 'Pi shutting down' if watch.get('action') == 'shutdown' else 'Pi would shut down (ACTION=log)'
    return watch['state']
