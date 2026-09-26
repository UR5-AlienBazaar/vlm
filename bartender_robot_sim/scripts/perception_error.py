#!/usr/bin/env python3
"""Measure stand-camera error_mm percentiles against Gazebo truth (server in compare mode)."""
import argparse
import json
import math
import random
import subprocess
import sys
import time
import urllib.request

import numpy as np

# Beer is not measured: moving it drags its cap's DetachableJoint. Slop is the stand clearance.
STATIONS = {
    'whiskey': ('jack_daniels_bottle', (0.08, -0.30), 0.0054),
    'cola': ('cola_bottle', (0.08, -0.15), 0.0087),
}
OFF_THE_BAR = (3.0, 3.0, 0.1)
SETTLE_S = 2.0
REMOVE_EVERY = 5


def set_pose(model, x, y, z):
    req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{w: 1}}')
    subprocess.run(
        ['ign', 'service', '-s', '/world/bar_world/set_pose',
         '--reqtype', 'ignition.msgs.Pose',
         '--reptype', 'ignition.msgs.Boolean',
         '--timeout', '3000', '--req', req],
        check=True, capture_output=True)


def read_world(url):
    with urllib.request.urlopen(url, timeout=5) as resp:
        doc = json.load(resp)
    if doc.get('perception') != 'compare':
        sys.exit(f'server is in {doc.get("perception")!r} mode; '
                 f'start it with --perception compare')
    return {s['id']: s for s in doc['stations']}


def random_in_disc(radius):
    r = radius * math.sqrt(random.random())
    a = random.uniform(0.0, 2.0 * math.pi)
    return r * math.cos(a), r * math.sin(a)


def place_bottles(victim):
    for name, (model, (x, y), slop) in STATIONS.items():
        if name == victim:
            set_pose(model, *OFF_THE_BAR)
        else:
            dx, dy = random_in_disc(slop)
            set_pose(model, x + dx, y + dy, 0.902)


def report(errors, wrong):
    print(f'\n{"station":8} {"n":>4} {"p50":>6} {"p95":>6} {"p99":>6} {"max":>6}')
    for name, errs in errors.items():
        stats = (np.percentile(errs, [50, 95, 99]).tolist() + [max(errs)]
                 if errs else [float('nan')] * 4)
        print(f'{name:8} {len(errs):4d} ' + ' '.join(f'{v:6.1f}' for v in stats))
    if wrong:
        sys.exit(f'FAIL: {len(wrong)} wrong readings (station, expected, got): '
                 f'{wrong}')
    print('\nOK: every in-stand bottle observed, every removed one missing '
          '(error_mm in mm).')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--trials', type=int, default=50)
    parser.add_argument('--url', default='http://127.0.0.1:8090/world')
    parser.add_argument('--seed', type=int, default=0)
    opts = parser.parse_args()
    random.seed(opts.seed)

    errors = {name: [] for name in STATIONS}
    wrong = []
    for trial in range(opts.trials):
        victim = (random.choice(sorted(STATIONS))
                  if trial % REMOVE_EVERY == REMOVE_EVERY - 1 else None)
        place_bottles(victim)
        time.sleep(SETTLE_S)
        stations = read_world(opts.url)
        for name in STATIONS:
            s = stations[name]
            expected = 'missing' if name == victim else 'observed'
            if s['observation'] != expected:
                wrong.append((name, expected, s['observation']))
            elif expected == 'observed' and s.get('error_mm') is not None:
                errors[name].append(s['error_mm'])
        print(f'trial {trial + 1}/{opts.trials}'
              + (f' ({victim} removed)' if victim else ''), file=sys.stderr)

    place_bottles(None)
    report(errors, wrong)


if __name__ == '__main__':
    main()
