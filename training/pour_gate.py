#!/usr/bin/env python3
"""Make a drink only if the glass stands where the pour was taught.

Run on the robot PC, beside `ros2 run bartender_api server --points workcell`:

    python3 pour_gate.py --teach 950,510    # glass on its spot; rough X,Y mm picks it out
    python3 pour_gate.py whiskey_cola       # checks the glass, then POSTs /make

The taught point is recorded from /circles/positions itself rather than from
the robot's pose, so any constant error in the camera's table calibration
cancels out: the check only asks "is the glass where it was when the pour
worked", not "where is it in base_link".
"""
import argparse
import json
import math
import time
import urllib.request
from pathlib import Path

TOLERANCE_MM = 20.0
MAX_AGE_S = 2.0
# The detector reports 7-11 circles a frame on the real table (arm joints, tape,
# caps) and most jump 40-300 mm frame to frame; the glass holds within ~10 mm.
# Demanding a match in every one of several readings is what filters them.
READINGS = 5


def _nearest(body, xy):
    circles = body.get("circles") or []
    return min(((c["x_mm"], c["y_mm"]) for c in circles), key=lambda c: math.dist(c, xy), default=None)


def check(bodies, taught, now, tolerance_mm=TOLERANCE_MM, max_age_s=MAX_AGE_S):
    """Reason to refuse the pour, or None if every reading has a circle within tolerance of `taught`."""
    worst = 0.0
    for body in bodies:
        # updated_at is stamped on the detector's machine; clock skew between it and
        # this one shows up as a false refusal, which is the safe direction.
        age = now - body.get("updated_at", 0)
        if not 0 <= age <= max_age_s:
            return f"circle reading is {age:.1f}s old (max {max_age_s}s); is circle_detect_live running?"
        glass = _nearest(body, taught)
        if glass is None:
            return "no glass seen on the table"
        worst = max(worst, math.dist(glass, taught))
    if worst > tolerance_mm:
        return f"glass is {worst:.0f} mm from the taught pour point (max {tolerance_mm:.0f} mm); move it back"
    return None


def taught_point(bodies, near, tolerance_mm=TOLERANCE_MM):
    """Average position of the circle nearest `near` across readings; refuses if it is missing or wanders."""
    hits = [_nearest(b, near) for b in bodies]
    if None in hits or max(math.dist(h, near) for h in hits) > 3 * tolerance_mm:
        raise ValueError(f"no circle within {3 * tolerance_mm:.0f} mm of {near} in every reading")
    x, y = (sum(v) / len(hits) for v in zip(*hits))
    if max(math.dist(h, (x, y)) for h in hits) > tolerance_mm / 2:
        raise ValueError("that circle moves between readings; is it really the glass?")
    return round(x, 1), round(y, 1)


def read_circles(count, timeout_s):
    """The next `count` /circles/positions messages published after subscribing (fewer on timeout)."""
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("pour_gate")
    got = []
    node.create_subscription(String, "/circles/positions", lambda msg: got.append(msg.data), 10)
    deadline = time.monotonic() + timeout_s
    while len(got) < count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return [json.loads(g) for g in got]


def teach(point_path, bodies, near):
    try:
        x, y = taught_point(bodies, near)
    except ValueError as error:
        raise SystemExit(f"not taught: {error}")
    point = {"x_mm": x, "y_mm": y}
    point_path.parent.mkdir(parents=True, exist_ok=True)
    point_path.write_text(json.dumps(point, indent=2))
    print(f"taught pour point {point['x_mm']:.0f},{point['y_mm']:.0f} mm -> {point_path}")


def make(api, drink):
    request = urllib.request.Request(f"{api}/make", json.dumps({"drink": drink}).encode(),
                                     {"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=600) as response:
        return response.read().decode()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("drink", nargs="?", help="menu key, e.g. whiskey_cola")
    parser.add_argument("--teach", metavar="X,Y", help="record the glass nearest this rough mm position as the pour point")
    parser.add_argument("--point", type=Path, default=Path("calibration/pour_point.json"))
    parser.add_argument("--tolerance-mm", type=float, default=TOLERANCE_MM)
    parser.add_argument("--api", default="http://127.0.0.1:8090")
    parser.add_argument("--wait", type=float, default=3.0, help="seconds to wait for a circle reading")
    args = parser.parse_args()
    if not args.teach and not args.drink:
        parser.error("name a drink, or pass --teach")

    bodies = read_circles(READINGS, args.wait)
    if len(bodies) < READINGS:
        raise SystemExit(f"refused: {len(bodies)}/{READINGS} readings on /circles/positions within {args.wait}s; "
                         "is circles_ros_relay running?")
    if args.teach:
        return teach(args.point, bodies, tuple(map(float, args.teach.split(","))))
    if not args.point.exists():
        raise SystemExit(f"refused: no taught pour point at {args.point}; run with --teach first")
    point = json.loads(args.point.read_text())
    why = check(bodies, (point["x_mm"], point["y_mm"]), time.time(), args.tolerance_mm)
    if why:
        raise SystemExit(f"refused: {why}")
    print(make(args.api, args.drink))


if __name__ == "__main__":
    main()
