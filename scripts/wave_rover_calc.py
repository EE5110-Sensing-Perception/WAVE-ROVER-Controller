#!/usr/bin/env python3
"""Joystick -> cmd_vel -> L/R motor calculator for wave_rover_bridge.

Full chain:
  joy_axis [-1..1]  x scale_linear / scale_angular
  -> Twist (linear.x, angular.z)
  -> bridge: L/R = (lin_x +/- ang_z * track/2) / max_speed  clamped [-1,1]

Usage:
  wave_rover_calc.py                                      # reads both YAMLs
  wave_rover_calc.py <max_speed> <track_width> <scale_lin> <scale_ang>
  wave_rover_calc.py <max_speed> <track_width> <scale_lin> <scale_ang> <joy_lin> <joy_ang>
"""

import os
import re
import sys

# ── math ──────────────────────────────────────────────────────────────────────
def compute_lr(lin_x, ang_z, max_spd, track):
    l = (lin_x - ang_z * track / 2.0) / max_spd
    r = (lin_x + ang_z * track / 2.0) / max_spd
    return l, r, max(-1.0, min(1.0, l)), max(-1.0, min(1.0, r))

def sat_threshold_joy(max_spd, track, scale_ang, limit):
    """Joystick fraction [0..1] at which a pure turn hits +-limit."""
    ang_z_sat = 2.0 * limit * max_spd / track if track > 0 else float('inf')
    return ang_z_sat / scale_ang if scale_ang > 0 else float('inf')

# ── YAML loaders ──────────────────────────────────────────────────────────────
def _find(text, key, default):
    m = re.search(rf'{key}\s*:\s*([0-9.]+)', text)
    return float(m.group(1)) if m else default

def load_bridge_yaml(path):
    with open(path) as f:
        text = f.read()
    return _find(text, 'max_speed', 1.25), _find(text, 'track_width', 0.15)

def load_teleop_yaml(path):
    with open(path) as f:
        text = f.read()
    return _find(text, r'scale_linear:\s*\n\s*x', 0.5), _find(text, r'scale_angular:\s*\n\s*yaw', 0.5)

# ── display ───────────────────────────────────────────────────────────────────
def bar(v_clamped, v_raw, width=28):
    b = ['.'] * width
    h, q = width // 2, width // 4
    for m in (q, h, h + q):
        b[m] = '|'
    pos = min(width - 1, max(0, round((v_clamped + 1.0) / 2.0 * (width - 1))))
    b[pos] = '#'
    clipped = abs(v_raw) > 1.0
    warn    = not clipped and abs(v_clamped) > 0.5
    tag = 'CLIP' if clipped else ('warn' if warn else '    ')
    return f"[{''.join(b)}] {tag}"

def row(joy_lin, joy_ang, ms, tw, sl, sa):
    lin_x = joy_lin * sl
    ang_z = joy_ang * sa
    l_raw, r_raw, l, r = compute_lr(lin_x, ang_z, ms, tw)
    return (f"  joy({joy_lin:+.2f},{joy_ang:+.2f})"
            f" vel({lin_x:+.3f},{ang_z:+.3f})"
            f"  L={l:+.3f} {bar(l, l_raw)}"
            f"  R={r:+.3f} {bar(r, r_raw)}")

# ── main ──────────────────────────────────────────────────────────────────────
def main():
    args  = sys.argv[1:]
    _here = os.path.dirname(os.path.abspath(__file__))
    bpath = os.path.normpath(os.path.join(_here, '..', 'config', 'wave_rover_bridge.yaml'))
    tpath = os.path.normpath(os.path.join(_here, '..', 'config', 'teleop_joy.yaml'))

    if len(args) == 0:
        ms, tw = load_bridge_yaml(bpath)
        sl, sa = load_teleop_yaml(tpath)
        print(f'(bridge: {os.path.basename(bpath)},  teleop: {os.path.basename(tpath)})')
    elif len(args) >= 4:
        ms, tw, sl, sa = float(args[0]), float(args[1]), float(args[2]), float(args[3])
    else:
        print(__doc__); sys.exit(1)

    print(f'\nmax_speed={ms}  track_width={tw}  scale_lin={sl}  scale_ang={sa}')
    print(f'  Max cmd_vel from full stick:  lin={sl:.3f} m/s  ang={sa:.3f} rad/s')

    j05 = sat_threshold_joy(ms, tw, sa, 0.5)
    j10 = sat_threshold_joy(ms, tw, sa, 1.0)
    print(f'  Pure-turn joy fraction -> L/R=+/-0.5 : {j05:.2f}  ({"OK - never" if j05 > 1.0 else f"at {j05*100:.0f}% stick"})')
    print(f'  Pure-turn joy fraction -> L/R=+/-1.0 : {j10:.2f}  ({"OK - never" if j10 > 1.0 else f"at {j10*100:.0f}% stick"})')

    if len(args) == 6:
        jl, ja = float(args[4]), float(args[5])
        print(f'\n{row(jl, ja, ms, tw, sl, sa)}')
        return

    print('\n-- forward sweep (joy_ang=0) --')
    for jl in (0.25, 0.5, 0.75, 1.0):
        print(row(jl, 0.0, ms, tw, sl, sa))

    print('\n-- turn sweep (joy_lin=0) --')
    for ja in (0.25, 0.5, 0.75, 1.0):
        print(row(0.0, ja, ms, tw, sl, sa))

    print('\n-- arc sweep (joy_lin=0.5) --')
    for ja in (0.25, 0.5, 0.75, 1.0):
        print(row(0.5, ja, ms, tw, sl, sa))


if __name__ == '__main__':
    main()
