#!/usr/bin/env python3
"""Generate the bottle stands -- the wells the bottles are returned into.

WHY
---
A bottle standing free on the counter tips at a purely geometric angle,
atan(half_width / com_height): 27 degrees for the whiskey, 17 for the cola.
Mass does not change that number (it cancels), which is why making the bottles
heavier did not stop them going over -- see the inertial comments in the two
model.sdf files. The only thing that raises the threshold is geometry, so this
adds the geometry: a shallow well the bottle base sits inside.

With the base captured, tipping is no longer a rotation the bottle can perform
at all until it has first been LIFTED clear of the rim. That converts a 17
degree tolerance into a 18mm one, and 18mm is far more than anything in the
place-and-release sequence produces.

It also catches the specific failure that has been in this system since before
the bottles were reweighed: the cola is placed upright and correctly and then
flung ~42mm as the pads spring apart, which puts it on its side. 42mm of
travel into a wall 9mm away is a nudge instead of a throw.

NO FLOOR, ON PURPOSE
--------------------
The well is a wall and nothing else. The counter is its floor.

This is the constraint the whole design is built around: bartender_pour's
Bottle.grasp_height is measured "up from the bottle's base" and used directly
as a base_link z, which is only correct while the base sits at z=0. Put so
much as a 3mm tray floor under the bottle and every grasp in the system is
3mm low, with no error message -- the pads simply close a little further down
the body than they were tuned to. A pedestal would mean re-tuning both
bottles; a bottomless ring means nothing above it has to change at all.

BOXES, NOT A MESH
-----------------
A ring is concave, and this project has already paid for that lesson once. The
grooved fingertip pad was built as a single concave mesh and did nothing,
because DART/bullet collides a mesh as its CONVEX HULL -- which filled the
groove straight back in (see PAD_MODE 'groove' in
bartender_description/scripts/render_bartender_urdf.py). The hull of a ring is
a solid disc, so a ring mesh here would not capture the bottle, it would block
it from ever being put down.

So the wall is a ring of overlapping boxes, each one convex. Sixteen of them
put the polygon's inner face within 2% of the nominal radius, which is well
inside the clearances below.

THE LEAD-IN
-----------
The top course of boxes steps outward by `lead`, giving the rim a chamfer. The
bottle is lowered straight down into the well, so any xy error at that moment
lands the base on the rim rather than in the well, and it would sit there
tipped. The step means an error up to (clearance + lead) is funnelled in
instead of parked on the edge.
"""
import argparse
import math
import os

# Bottle base dimensions come from models/*/model.sdf.
#
#   whiskey: a 0.0772 square box, so its base needs a circle big enough for
#            the CORNERS (0.0546) whatever yaw it comes back at. A square
#            socket would hold yaw too, but it would also refuse a bottle
#            returned a few degrees off -- and coming back yawed is a thing
#            this bottle measurably does.
#   cola:    a 0.0333 cylinder, and round, so yaw never matters.
#   beer:    a 0.0322 cylinder, round as well. It gets the tightest clearance
#            of the three, and deliberately: it is the one bottle that is
#            pushed DOWN on while it stands here, by the opener in the other
#            arm, and the well is what stops that push walking it sideways off
#            the station. Nothing else in the cycle loads a bottle this way.
#
# `clearance` is slop between the bottle and the wall at the tightest yaw. It
# trades two failures against each other: too tight and a slightly misplaced
# bottle is dropped onto the rim, too loose and the bottle has room to build
# up speed before the wall stops it. The lead-in chamfer above covers the
# first, which is what allows these to be as tight as they are.
STANDS = {
    'whiskey_stand': dict(
        bottle_radius=0.0546, clearance=0.0054,
        note='well for the square Jack Daniels bottle; sized on its corners'),
    'cola_stand': dict(
        bottle_radius=0.0333, clearance=0.0087,
        note='well for the round cola bottle'),
    'beer_stand': dict(
        bottle_radius=0.0322, clearance=0.0078,
        note='well for the round beer bottle'),
}

WALL = 0.008        # radial thickness of the wall
HOLD_H = 0.018      # height of the straight part -- what actually holds
LEAD_H = 0.008      # height of the chamfered lead-in above it
LEAD = 0.006        # how far the lead-in steps outward
SEGMENTS = 16

# Rubber, so a bottle that touches the wall stops rather than skating along
# it. The bottles are 0.9 and the pads 1.8; this sits between them.
STAND_MU = 1.2


def courses(inner_r):
    """Return the two stacked rings: (inner radius, z of its base, height)."""
    return [(inner_r, 0.0, HOLD_H),
            (inner_r + LEAD, HOLD_H, LEAD_H)]


def segment_poses(inner_r, segments=SEGMENTS):
    """Place each wall segment of one course: box centre (x, y) and yaw.

    Each box is laid with its inner face tangent to the circle of radius
    `inner_r`, so the polygon's INRADIUS is exactly `inner_r` -- the number
    the clearances are quoted against. Its length is taken from the outer
    tangent circle, which makes neighbouring boxes overlap slightly at the
    corners instead of leaving a gap the size of the wall.
    """
    length = 2.0 * (inner_r + WALL) * math.tan(math.pi / segments)
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        r = inner_r + WALL / 2.0
        yield (r * math.cos(angle), r * math.sin(angle), angle, length)


def boxes(inner_r, segments=SEGMENTS):
    """Yield every (xyz, yaw, size) in the model, both courses."""
    for course_r, z0, height in courses(inner_r):
        for x, y, yaw, length in segment_poses(course_r, segments):
            yield ((x, y, z0 + height / 2.0), yaw, (WALL, length, height))


def model_sdf(name, spec, segments=SEGMENTS):
    inner_r = spec['bottle_radius'] + spec['clearance']
    top_r = inner_r + LEAD + WALL
    parts = []
    for i, (xyz, yaw, size) in enumerate(boxes(inner_r, segments)):
        pose = '{:.6f} {:.6f} {:.6f} 0 0 {:.6f}'.format(*xyz, yaw)
        dims = '{:.6f} {:.6f} {:.6f}'.format(*size)
        # Collisions are left UNNAMED for the same reason the fingertip pad
        # boxes are: sdformat matches <gazebo reference> surface properties
        # against collisions by name and the match fails silently when they
        # carry one. This model sets friction inline rather than through a
        # reference, but keeping the convention means it survives being
        # converted or re-tagged later.
        parts.append(f"""
      <visual name="wall_{i}">
        <pose>{pose}</pose>
        <geometry><box><size>{dims}</size></box></geometry>
        <material>
          <ambient>0.10 0.10 0.12 1</ambient>
          <diffuse>0.16 0.16 0.19 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
      <collision name="wall_collision_{i}">
        <pose>{pose}</pose>
        <geometry><box><size>{dims}</size></box></geometry>
        <surface>
          <friction><ode>
            <mu>{STAND_MU}</mu>
            <mu2>{STAND_MU}</mu2>
          </ode></friction>
        </surface>
      </collision>""")

    return f"""<?xml version="1.0" ?>
<!-- GENERATED by ros2_ws/src/bartender_gazebo/scripts/make_bottle_stands.py.
     Edit that script and re-run it; edits here are overwritten. -->
<sdf version="1.9">
  <model name="{name}">
    <!-- {spec['note']}.

         Static, and BOTTOMLESS: the counter is the floor. The bottle base
         therefore still rests at the same height it always did, which is what
         keeps bartender_pour's grasp_height (measured up from the base, used
         directly as a base_link z) correct without any change.

         Geometry, all in metres:
           bottle radius   {spec['bottle_radius']:.4f}  (corners, if it is square)
           clearance       {spec['clearance']:.4f}
           inner radius    {inner_r:.4f}   holds the bottle over z 0..{HOLD_H:.3f}
           lead-in         +{LEAD:.4f} over z {HOLD_H:.3f}..{HOLD_H + LEAD_H:.3f}
           outer radius    {top_r:.4f}   <- publish THIS to the planning scene
           wall            {WALL:.4f} thick, {segments} segments

         Link origin is the centre of the well at counter level, so the model
         pose is simply the bottle's own station pose. -->
    <static>true</static>
    <link name="stand_link">{''.join(parts)}
    </link>
  </model>
</sdf>
"""


def config(name, spec):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{spec['note']}. Generated by make_bottle_stands.py.</description>
</model>
"""


def outer_radius(name):
    """Return what the planning scene should use. Imported by the tests."""
    spec = STANDS[name]
    return spec['bottle_radius'] + spec['clearance'] + LEAD + WALL


def total_height():
    return HOLD_H + LEAD_H


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.normpath(
        os.path.join(here, *([os.pardir] * 4), 'models'))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default=default_out,
                    help='models/ directory to write into')
    ap.add_argument('--segments', type=int, default=SEGMENTS)
    args, _ = ap.parse_known_args()

    for name, spec in STANDS.items():
        path = os.path.join(args.out, name)
        os.makedirs(path, exist_ok=True)
        with open(os.path.join(path, 'model.sdf'), 'w') as fh:
            fh.write(model_sdf(name, spec, args.segments))
        with open(os.path.join(path, 'model.config'), 'w') as fh:
            fh.write(config(name, spec))
        print(f'{path}: inner {spec["bottle_radius"] + spec["clearance"]:.4f} '
              f'outer {outer_radius(name):.4f} '
              f'height {total_height():.3f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
