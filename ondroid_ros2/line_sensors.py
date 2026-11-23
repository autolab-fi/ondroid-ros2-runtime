"""Eight-channel Octoliner data and the existing Metropolia track fixture.

Channels run from the robot's right (0) to left (7). Positive normalized error
means the line is left of centre, matching positive ROS angular.z. Simulator
ADC values and sensor geometry are illustrative, not per-board calibration.
"""
import base64
from functools import lru_cache
import json
import math
from pathlib import Path
import zlib


def analyse(raw):
    if (not isinstance(raw, (list, tuple)) or len(raw) != 8
            or any(type(v) is not int or not 0 <= v <= 4095 for v in raw)):
        raise ValueError('Line sensors require eight ADC integers from 0 to 4095')
    lo, hi = min(raw), max(raw)
    active = [hi-lo >= 80 and v >= (lo+hi)/2 for v in raw]
    pattern = sum((1 << (7-i)) for i, seen in enumerate(active) if seen)
    weights = [max(0., (v-lo)/(hi-lo)-.5) if hi-lo >= 80 else 0. for v in raw]
    total = sum(weights)
    error = sum(((i-3.5)/3.5)*w for i, w in enumerate(weights))/total if total else None
    return dict(raw=list(raw), pattern=pattern, line_visible=bool(total), error=error)


@lru_cache(maxsize=1)
def track():
    spec = json.loads((Path(__file__).with_name('assets')/'metropolia-line.json').read_text())
    mask = zlib.decompress(base64.b64decode(spec['mask_zlib_base64']))
    if len(mask) != (spec['width']*spec['height']+7)//8:
        raise ValueError('Invalid line-track fixture')
    return spec, mask


def on_line(x, y):
    spec, mask = track()
    if not 0 <= x < spec['world_width_m'] or not -spec['world_height_m'] < y <= 0:
        return False
    ix = min(spec['width']-1, int(x/spec['world_width_m']*spec['width']))
    iy = min(spec['height']-1, int(-y/spec['world_height_m']*spec['height']))
    index = iy*spec['width']+ix
    return bool(mask[index//8] & (1 << (index%8)))


def simulated(x, y, yaw):
    raw = []
    for i in range(8):
        lateral = -.035+i*.07/7
        sx = x+.10*math.cos(yaw)-lateral*math.sin(yaw)
        sy = y+.10*math.sin(yaw)+lateral*math.cos(yaw)
        raw.append(960 if on_line(sx, sy) else 100)
    return analyse(raw)
