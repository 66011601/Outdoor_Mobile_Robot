import math
import os

NAV_CMD_FILE = '/tmp/dcvg_nav_cmd'
STOP_STATUS_FILE = '/tmp/dcvg_stop_status'
STOP_CONFIRM_FILE = '/tmp/dcvg_stop_confirm'
NAV_TARGET_FILE = '/tmp/dcvg_nav_target'


def read_file(path, default=''):
    try:
        with open(path) as f:
            return f.read().strip()
    except FileNotFoundError:
        return default


def write_file(path, text):
    with open(path, 'w') as f:
        f.write(text)


def remove_file(path):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass


def read_cmd():
    return read_file(NAV_CMD_FILE, 'run')


def write_cmd(cmd):
    write_file(NAV_CMD_FILE, cmd)


def haversine_distance(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin(math.radians(lat2 - lat1) / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6_371_000 * math.asin(math.sqrt(a))
