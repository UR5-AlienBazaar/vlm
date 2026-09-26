#!/usr/bin/env python3
"""Render bartender.urdf.xacro with the fingertip pads this project needs.

Sets the pad friction, widens the pads, and can cut a cylinder-holding
V-groove into them instead.

READ THIS FIRST: the groove is built and works, but it is OFF by default
because it regresses the whiskey bottle. The reasoning, the measurements and
the known fix are all at PAD_MODE below. Everything after this paragraph
describes the groove itself.

WHY THIS EXISTS
---------------
The Robotiq 2F-85 fingertips are flat pads, and a flat pad cannot hold a
smooth cylinder. Measured on the cola bottle: a flat pad needs the knuckle to
close ~6mm past first touch before the joint stalls, because the bottle is
being squeezed out from between the pads rather than stopping them. At that
depth it escapes sideways -- one traced run spun the bottle 111 degrees and
threw it off the counter. The square whiskey bottle has none of this trouble:
flat on flat stalls within 0.2mm.

The fix is a V-groove running along the bottle's axis, which constrains the
bottle in the one direction it was escaping. Friction still carries the
bottle's weight (that is along the groove, which a groove cannot help with),
so the pads also get a rubber-like friction coefficient here -- see the
GAZEBO TAGS note below for why that has to be set in this file.

WHY IT IS A SCRIPT AND NOT XACRO
--------------------------------
The groove has to REPLACE the stock fingertip collision, not add to it: the
stock mesh is the innermost surface, so any pad merely bolted on top would
never touch the bottle. xacro can add to an included macro but cannot remove
from one, and robotiq_description is a system package we do not want to fork.
So this runs xacro and rewrites the two fingertip collisions in the result.
Both sim.launch.py and move_group.launch.py call this instead of xacro, so
Gazebo and MoveIt always see the same robot.

WHY THE GROOVE IS CUT IN, NOT ADDED ON
--------------------------------------
This is forced, not a preference. The pads open to exactly 85.00mm and the
whiskey body is 77.2mm across, leaving 3.9mm a side. Pads that protrude
inward eat that clearance, and they have to protrude a long way to be useful:
to reach a 59.6mm cylinder at the +/-14mm flank offset they would need 4.5mm,
which puts the opening needed for the whiskey at 86.2mm -- more than the
gripper physically has. Cutting the groove back into the existing envelope
instead keeps the maximum opening at 85.00mm, so the approach clearance is
unchanged, and it leaves the pad flat outside the groove, so the whiskey
still gets full face contact on a 28mm band and its clamp is untouched.

GAZEBO TAGS
-----------
bartender.urdf.xacro used to carry a note saying <gazebo reference> friction
tags "do nothing" on this spawn path. That was over-read from a null result.
Checked by converting with `ign sdf -p`: the <surface> blocks that ship inside
robotiq_description's <collision> elements are indeed dropped, because URDF
has no such element, but <gazebo reference="link"> blocks convert correctly
and land as <surface><friction><ode><mu>. The original experiment swept mu
during the old overhead NECK grasp, which was held geometrically by the
fingers wrapping the neck -- friction genuinely could not have mattered there,
whatever the tags did.
"""
import json
import math
import os
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

# Matched by SUFFIX, not by equality: the description carries two grippers
# now and the second one's links are all under a `b_` tf_prefix (see the
# xacro's "WHY ONE ARM IS PREFIXED" note). Every gripper in the model gets the
# same pads and the same friction -- there is no case where one arm should be
# holding bottles with a different fingertip from the other, and if there ever
# is, it wants a per-arm PAD_MODE rather than a name list.
TIPS = ('robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link')


def is_fingertip(link_name):
    return any(link_name.endswith(tip) for tip in TIPS)


# BARTENDER_PAD_MODE:
#   wide   - stock meshes PLUS a wing above and below the pad, widening the
#            contact face from 22mm to PAD_WIDTH along the bottle's axis
#            (DEFAULT, what ships). See WHY THE PADS ARE WIDENED below.
#   mesh   - stock meshes untouched, only friction set. The control for 'wide'
#   boxes  - primitive pads carrying the V-groove; the groove works, but read
#            the trade-off below before switching, and set COLA's clamp_pos to
#            0.55 in bartender_pour (its stall moves from ~0.28 to ~0.39)
#   flat   - primitive pads, no groove; the control that isolated the two
#   hook   - replace the pad entirely with the neck-hooking fingertip from
#            ../../../../../fingertip, which carries the bottle on a ledge
#            under its collar instead of on pad friction. Needs bottles that
#            HAVE a collar; see the note in that project's README.
#   groove - stock mesh with the V cut into its pad face. DOES NOT WORK, kept
#            because it is not obvious why: for a dynamic body DART/bullet
#            collides against the mesh's CONVEX HULL, which fills the groove
#            straight back in. Traced, the knuckle runs to 0.503 and the
#            bottle is flung, where the same groove built from boxes stalls
#            stably at 0.369-0.393. A concave pad has to be several convex
#            pieces; it cannot be one concave mesh.
#
# WHY THE DEFAULT IS 'mesh' AND NOT THE GROOVE
# --------------------------------------------
# The groove does what it was built to do. On flat pads the cola bottle never
# stalls the joint at all -- it is squeezed out and the knuckle runs to the
# commanded clamp -- and with the grooved boxes it stalls at 0.369 and holds
# 0.37-0.39 through the lift, the pour and the return. Full drinks went from
# about 3/8 to 6/12.
#
# It costs the whiskey, though, and that is not a fair trade. The groove has
# to sit where the cola's axis crosses the pad (local z 0.0374), and the
# whiskey's axis crosses at 0.0439 -- inside the groove. So the whiskey ends
# up supported only below its centreline plus two wing lines, and it pivots:
# over 12 runs it came back yawed more than the 6 degrees that still allows a
# re-grasp in 4 of them, against 0 of 8 on stock pads. A bottle that cannot be
# picked up next time is worse than a mixer that sometimes is not poured.
#
# The fix is known and is NOT more groove tuning: give Bottle its own
# grip_ahead instead of the global GRIP_AHEAD_OF_TOOL0, and grip the cola
# ~10mm deeper (0.135). That moves the cola's axis to local z ~0.0274, so the
# groove moves with it toward the palm and leaves flat pad at the front, at
# 0.041-0.051, which is exactly where the whiskey's axis sits. Both bottles
# then get what they need from the same pad. It needs a new IK seed for the
# cola's pre-grasp, which is why it is not done here.
# WHY THE PADS ARE WIDENED, AND IN WHICH DIRECTION
# ------------------------------------------------
# Only one direction was available, and it is worth writing down which, since
# the other two are closed for reasons that are easy to rediscover the hard
# way.
#
# In the fingertip's frame x is the closing axis, y runs along the grasped
# bottle's axis (vertical, in the side grasp) and z is the depth into the
# grip. The stock pad is 22mm in y by 38mm in z.
#
#   x -- INWARD IS FORBIDDEN. The pads open to 85.00mm and the whiskey is
#        77.2mm across, so there is 3.9mm a side. Anything protruding into the
#        grip spends that clearance, and the approach has none to spare.
#
#   z -- FULL, BOTH WAYS. The pad already runs to local z 51.01, which is the
#        very end of the fingertip, so there is nothing ahead of it. Behind it
#        is worse than unavailable: extending toward the palm is a MEASURED
#        regression, recorded in real_pad_face() below, where a pad spanning
#        the whole bounding box moved the contact centroid from 12mm behind
#        the bottle's axis to 21mm behind it and the whiskey pitched out of
#        the grip on every pour.
#
#   y -- FREE, and the useful one anyway. This is the direction the grip is
#        weakest in, because it is the moment arm that resists the bottle
#        PITCHING out of the fingers -- the exact failure above. The stock
#        pads share 21.15mm, i.e. +/-10.6mm; PAD_HALF_WIDTH gives +/-15.5mm.
#
# HOW WIDE, AND WHAT SETS THE LIMIT
# ---------------------------------
# The bottles do, not the gripper, and the limit is DERIVED below rather than
# asserted, because it moved once already: this used to read "the cola's waist
# is the binding constraint" and that stopped being true the day the beer
# arrived.
#
# The pad's span along the bottle is measured, not assumed. FK along the
# knuckle chain is a pure translation in x and z -- every joint from tool0
# down to the fingertips has y = 0 in its origin and (0, -1, 0) for its axis
# -- so a fingertip's LOCAL y is tool0's y exactly, at every knuckle angle
# from open through both clamps to fully closed. The 2F-85's parallelogram
# moves the pads in x and z only. The pad therefore never changes height
# relative to the bottle, and the margins below are static.
#
# THE TWO STOCK PADS ARE NOT MIRROR IMAGES. Measured off the shipped meshes:
# the left pad runs -11.80..+10.20mm about tool0 and the right runs
# -10.95..+11.05, so they are 0.85mm out of register and the band where BOTH
# touch is 21.15mm, not the 22mm either one is. That is not a rounding
# artefact in robotiq_description, it is in the geometry, and until the wings
# were made to land on a common band it meant every grasp carried a small
# couple: each pad overhung the other at one end and was overhung at the
# other. widen_pad() now grows each side to +/-PAD_HALF_WIDTH about tool0
# rather than symmetrically about its own centre, so the two pads register
# and the full width is contact rather than width-minus-0.85.
PAD_LANDING_MARGIN = 0.0035

# Where each bottle is actually gripped, and what the bottle does at each end
# of that band. Heights are up from the bottle's base, in metres, and the
# bands are the collision cylinders in models/<bottle>/model.sdf rather than
# anything approximate; grasp heights are bartender_pour's WHISKEY/COLA
# grasp_height and bartender_open's BEER_GRASP_HEIGHT.
#
# The last two flags are the point of the table. A band edge only constrains
# the pad if the bottle steps OUTWARD there, because then a pad reaching past
# it lands on the step and never touches the band at all:
#
#   whiskey  77.2mm square body over 0.000..0.150, and above it a 50mm neck.
#            Narrower. Neither edge blocks; the whiskey would take a 55mm pad.
#   cola     59.6mm waist over 0.040..0.080 between a 66.6mm base and a
#            67.2mm bulge. BOTH edges block, and this 40mm window is what
#            used to set the limit on its own.
#   beer     38.7mm neck over 0.170..0.200 above a 51.0mm shoulder, with a
#            34.9mm taper above. The shoulder blocks; the taper does not.
#
# The beer's shoulder is not a theoretical limit. With the pad at 0.173 it
# was traced stalling dead at a 41.6mm gap and holding there through fifteen
# further commands -- a stable grip, on the shoulder step, not on the neck.
# See BEER_GRASP_HEIGHT in bartender_open/layout.py for the trace.
GRIP_BANDS = (
    # name, band low, band high, grasp height, low edge blocks, high blocks
    ('whiskey', 0.000, 0.150, 0.1225, False, False),
    ('cola', 0.040, 0.080, 0.0600, True, True),
    ('beer', 0.170, 0.200, 0.1890, True, False),
)


def pad_half_width_limit(bands=GRIP_BANDS, margin=PAD_LANDING_MARGIN):
    """How far the pad may reach either side of tool0, over all the bottles.

    The margin is for where the arm actually lands, not for the bottle:
    Cartesian moves here arrive within about 3mm of the commanded height
    (measured on the teach pendant -- a 50mm jog moved 47mm), so a pad edge
    closer than that to a blocking step will sometimes be past it.
    """
    reach = []
    for _name, low, high, grasp, low_blocks, high_blocks in bands:
        if low_blocks:
            reach.append(grasp - low - margin)
        if high_blocks:
            reach.append(high - grasp - margin)
    return min(reach)


# 31mm, which is the derived limit above and not a round number chosen first.
# The three blocking edges come out at 16.5mm (cola below), 16.5mm (cola
# above) and 15.5mm (beer's shoulder), so the BEER binds -- which is the
# whole reason this is a derivation and not a constant. It was the cola until
# a capped bottle was added to the scene.
#
# 33mm WAS TRIED AND DID NOT SURVIVE, and it is worth saying how, because the
# extra 2mm is sitting right there and looks free. It is reachable only by
# gripping the beer 2mm further up its neck, at 0.191, which moves the whole
# pad up with it. Both clean runs it got failed, and the grasp was measurably
# deeper with it -- the fingers closing 6.5 and 6.6mm into a 38.7mm neck,
# against 1.0-5.3mm over the eight runs at 0.189 that opened 6. Two runs do
# not disprove a configuration and this is not claiming they do; they were
# enough to stop spending simulator time on it while the conservative option
# was untested.
#
# So the beer stays gripped at 0.189, where it was tuned, and the pad takes
# what that leaves. GRIP_BANDS carries a copy of that height and
# bartender_open's test_layout.py checks the pair have not drifted apart.
#
# What this is worth, and what it is not: against the 21.15mm both stock pads
# really share, 33mm registered is a little over half as much again of moment
# arm resisting the bottle PITCHING out of the fingers, which is the measured
# whiskey failure. It buys no straight pull-out resistance at all -- Coulomb
# friction is mu times the normal force and does not depend on contact area
# -- and against the round cola it does not even lengthen the contact, since
# a flat pad on a cylinder touches along a line whatever its width. All of
# the gain is torque. If what is wanted is a bottle that cannot be pulled out
# of the fingers, the answer is PAD_MODE 'hook', not a wider pad.
PAD_HALF_WIDTH = 0.0155
PAD_WIDTH = 2.0 * PAD_HALF_WIDTH

PAD_MODE = os.environ.get('BARTENDER_PAD_MODE', 'wide')

# Grooved meshes live in this package, generated by make_grooved_pads.py.
# Resolved from this file's own location rather than written as $(find ...),
# because xacro has already run by the time we substitute it and would not get
# another chance to expand it. Works the same from the source tree or from
# install/share, as long as CMakeLists.txt installs meshes/ beside scripts/.
MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes')
GROOVED_MESH = 'file://' + os.path.normpath(
    os.path.join(MESH_DIR, '{side}_finger_tip_grooved.stl'))

# The neck-hooking fingertip, generated by fingertip/fingertip.py into
# MESH_DIR along with a .json of its derived dimensions.
HOOK_MESH = os.path.normpath(os.path.join(MESH_DIR, 'fingertip_{side}.stl'))
HOOK_DIMS = os.path.normpath(os.path.join(MESH_DIR, 'fingertip_{side}.json'))

# Groove geometry, in fingertip-local metres. The fingertip frame stays
# axis-aligned with the gripper base through the whole closing sweep (the
# 2F-85's parallelogram linkage keeps the pads parallel), so these need no
# correction for how far the gripper happens to be closed.
#
# GROOVE_CENTRE_Z: where the bottle's axis actually crosses the pad. By FK
#   along the knuckle chain, the cola's axis sits at local z = 0.0374 once
#   seated and the whiskey's at 0.0439.
# GROOVE_HALF_HEIGHT: flank reach either side of centre. 0.0135 puts the upper
#   wing at 0.0509, just inside the pad's real top edge at 0.05101.
# GROOVE_DEPTH: 0.0045 gives flanks at 18.4 degrees from the pad plane. The
#   contact line then lands 9.4mm off centre for the cola's 59.6mm waist and
#   10.6mm for its 67.2mm bulge, both inside the 13.5mm flank, so the bottle
#   seats on the flanks and never rides over the wing. Deeper was tempting for
#   a stronger wedge, but it walks the contact out towards the rim.
GROOVE_CENTRE_Z = 0.0374
GROOVE_HALF_HEIGHT = 0.0135
GROOVE_DEPTH = 0.0045
FLANK_THICKNESS = 0.006

# How far the pad stands proud of the finger body behind it. The real pad is
# a raised face, not the whole fingertip -- see real_pad_face().
PAD_THICKNESS = 0.006

# Rubber pads on glass. The stock description asks for mu 100000, which is a
# Gazebo-Classic idiom for "never slip" and not a coefficient; DART takes it
# literally. The bottles use 0.9.
#
# 1.2 -> 1.8 when the bottles were given realistic masses (whiskey 1.0 ->
# 1.35kg, cola 0.8 -> 1.05kg). 1.2 had been tuned against bottles that weighed
# a third less, and at the 97 degree pour tilt gravity acts along the pad
# faces, so friction alone carries the whole weight.
#
# Measured, one full whiskey-and-coke per run, counting the drink completing
# and the whiskey ending back on its station:
#
#     bottles   PAD_MU   drinks poured   whiskey returned
#     1.0/0.8     1.2         3/3              3/3
#     1.35/1.05   1.2         3/6              4/7
#     1.35/1.05   1.8         7/7              6/7
#
# The middle row is the regression: the heavier bottles at the old friction
# lose the COLA mid-pour ("cola lost: fingers have closed to 0.44") in three
# runs of six, which never happens at 1.8. Small samples, but the direction
# matches the physics and the drink-completion column is not marginal.
#
# Friction rather than a harder squeeze: see pour_action_server's WHISKEY
# notes. The knuckle already stalls on the bottle, so a higher clamp command
# only raises the position error and the effort behind it.
#
# 1.8 against the bottles' 0.9 is soft rubber on glass. It is on the generous
# side of real, but far nearer it than the stock 100000, and it is the same
# kind of allowance the pads already had at 1.2.
#
# NOT fixed by any of this, and pre-existing: the cola bottle is left tipped
# over at the end of almost every run, in EVERY configuration above including
# the original one. It is knocked during the place-and-release, after the
# drink has already been poured, which is why the action still reports
# success. Worth its own investigation; it is not a consequence of the mass
# or friction change.
PAD_MU = 1.8


def stl_tris(path):
    data = open(path, 'rb').read()
    count = struct.unpack('<I', data[80:84])[0]
    out = []
    for i in range(count):
        base = 84 + i * 50 + 12
        out.append(tuple(struct.unpack('<fff', data[base + k * 12:base + k * 12 + 12])
                         for k in range(3)))
    return out


def stl_bbox(tris):
    lo, hi = [1e9] * 3, [-1e9] * 3
    for tri in tris:
        for v in tri:
            for a in range(3):
                lo[a] = min(lo[a], v[a])
                hi[a] = max(hi[a], v[a])
    return lo, hi


def real_pad_face(tris, inner_x):
    """Measure the actual rubber pad: the triangles in the contact plane.

    This matters more than it looks. The fingertip's BOUNDING BOX face is
    27 x 57mm, but the pad that really touches anything is 22 x 38mm and sits
    at local z +13.0..+51.0 -- the fingertip is relieved behind it. An earlier
    attempt here replaced the mesh with one box spanning the whole bbox, which
    invented 19mm of pad reaching back toward the palm and almost doubled the
    contact area. The whiskey bottle then slipped during the pour every time:
    the contact patch's centroid moved from 12mm behind the bottle's axis to
    21mm behind it, and the pouring torque pitched the bottle out of the grip.
    Traced, the knuckle crept from its 0.090 stall to 0.125 -- a 73mm gap on a
    77mm bottle, i.e. the pads had slid down onto the shoulder taper -- and
    the bottle was released on its side. Measuring the real face instead fixes
    that, and it is also why `simplify_pads.py`-style bbox substitution was a
    dead end when it was tried for speed.
    """
    ys, zs = [], []
    for tri in tris:
        if max(v[0] for v in tri) - min(v[0] for v in tri) > 1e-6:
            continue
        if abs(min(v[0] for v in tri) - inner_x) > 1e-5:
            continue
        ys += [v[1] for v in tri]
        zs += [v[2] for v in tri]
    if not ys:
        raise SystemExit('could not find the pad face in the fingertip mesh')
    return (min(ys), max(ys)), (min(zs), max(zs))


def widen_pad(link, tris):
    """Add a wing above and below the stock pad, on the pad's own plane.

    Both fingertips are grown to the SAME band, +/-PAD_HALF_WIDTH about
    tool0, rather than each being grown symmetrically about its own centre.
    That matters because the stock pads are not mirror images -- the left sits
    0.85mm lower than the right -- so growing each about itself preserves the
    mismatch, and only the overlap of the two is real contact. Growing both to
    a common band registers them: the wings come out slightly different sizes
    on the two sides, which is the asymmetry being cancelled rather than a
    bug. A fingertip's local y is tool0's y exactly; see the FK note above.

    ADDITIVE, on purpose. The stock mesh collision is left exactly as it is
    and the wings only extend it in y, so the contact that the whole system is
    already tuned against is untouched and the change is confined to the part
    that is new. Replacing the mesh with primitives is what 'flat' does, and
    it is a bigger change than "the pad is wider" should be.

    The wings sit ON the pad plane (x = inner), not proud of it, so the
    gripper's opening is unchanged to the micron. They also do not overlap the
    pad: each runs from a pad edge outward, meeting it along a line. Coplanar
    OVERLAPPING faces are what makes contact solvers chatter; abutting ones do
    not.

    One subtlety that makes this work at all: the fingertip is a dynamic body,
    so DART collides it as its CONVEX HULL, not as the mesh. That sounds like
    it would ruin the idea, and it is the trap that killed PAD_MODE 'groove'.
    Here it is harmless, and in fact helpful: the pad is the innermost feature
    of the fingertip, so the hull's inner face IS the pad face and nothing
    else -- the hull fills the relief BEHIND the pad, which never touches
    anything. That is also why the stock mesh has always behaved like a plain
    22x38 pad.
    """
    lo, hi = stl_bbox(tris)
    inner, outer = (lo[0], hi[0]) if abs(lo[0]) > abs(hi[0]) else (hi[0], lo[0])
    out = 1.0 if outer > inner else -1.0
    (pad_y0, pad_y1), (pad_z0, pad_z1) = real_pad_face(tris, inner)

    # Per-edge, to a common band. A stock edge already outside the band would
    # mean PAD_HALF_WIDTH is narrower than the stock pad, which cannot be
    # trimmed by adding geometry -- so say so rather than emitting a wing of
    # negative size.
    wings = ((pad_y0 - -PAD_HALF_WIDTH, -PAD_HALF_WIDTH, -1.0),
             (PAD_HALF_WIDTH - pad_y1, PAD_HALF_WIDTH, 1.0))
    if any(grow < 0.0 for grow, _edge, _sign in wings):
        raise SystemExit(
            f'PAD_HALF_WIDTH {PAD_HALF_WIDTH} is inside the stock pad face '
            f'({pad_y0:.5f}..{pad_y1:.5f}); a wing cannot remove material')
    for grow, edge, sign in wings:
        if grow <= 0.0:
            continue
        box(link, 'pad_wing',
            (PAD_THICKNESS, grow, pad_z1 - pad_z0),
            (inner + out * PAD_THICKNESS / 2.0,
             edge - sign * grow / 2.0,
             (pad_z0 + pad_z1) / 2.0))
    return wings


def hook_fingertip(link, tris, side):
    """Swap the stock pad for the neck-hooking fingertip.

    FRAMES. The fingertip is modelled in its own convention -- origin at the
    centre of the mounting face, +X toward the bottle, +Y across the finger,
    +Z up the bottle's axis -- and in millimetres. The Robotiq fingertip link
    is metres, with local x the closing axis, y up, and z along the finger.
    So the part's axes map to the link's as
        part +X -> link -x (left) or +x (right), i.e. inward toward the bottle
        part +Z -> link +y, up
        part +Y -> link +z (left) or -z (right), completing a right-handed set
    which is rpy (pi/2, 0, pi) on the left and (-pi/2, 0, 0) on the right.

    The part is then sat on the stock pad's own contact plane, centred on that
    pad, so the hook protrudes into the grip exactly as far as the generator
    says it does.
    """
    lo, hi = stl_bbox(tris)
    inner = lo[0] if abs(lo[0]) > abs(hi[0]) else hi[0]
    (pad_y0, pad_y1), (pad_z0, pad_z1) = real_pad_face(tris, inner)

    with open(HOOK_DIMS.format(side=side)) as fh:
        dims = json.load(fh)
    body_h = dims["body_h"] / 1000.0        # generator works in mm

    xyz = (inner,
           (pad_y0 + pad_y1) / 2.0 - body_h / 2.0,
           (pad_z0 + pad_z1) / 2.0)
    roll = math.pi / 2.0 if side == "left" else -math.pi / 2.0
    yaw = math.pi if side == "left" else 0.0

    for tag in ("collision", "visual"):
        for el in list(link.findall(tag)):
            link.remove(el)
        el = ET.SubElement(link, tag)
        origin = ET.SubElement(el, "origin")
        origin.set("xyz", " ".join(f"{v:.6f}" for v in xyz))
        origin.set("rpy", f"{roll:.6f} 0 {yaw:.6f}")
        geom = ET.SubElement(el, "geometry")
        mesh = ET.SubElement(geom, "mesh")
        mesh.set("filename", "file://" + HOOK_MESH.format(side=side))
        mesh.set("scale", "0.001 0.001 0.001")
    return dims


def box(parent, name, size, xyz, pitch=0.0):
    """Add one collision box.

    `name` documents the part but is deliberately NOT written as a name=
    attribute: sdformat matches <gazebo reference>
    surface properties against collisions by name, and giving them explicit
    names makes the match fail silently. Measured on this exact model --
    named collisions convert with zero <mu> elements, the identical geometry
    left unnamed converts with all eight. They come out as
    <link>_collision, _collision_1, ... in the order added here:
    flat pad, lower flank, upper flank, backing.
    """
    col = ET.SubElement(parent, 'collision')
    origin = ET.SubElement(col, 'origin')
    origin.set('xyz', ' '.join(f'{v:.6f}' for v in xyz))
    origin.set('rpy', f'0 {pitch:.6f} 0')
    geom = ET.SubElement(col, 'geometry')
    ET.SubElement(geom, 'box').set('size', ' '.join(f'{v:.6f}' for v in size))


def groove_pad(link, tris):
    """Replace a fingertip's mesh collision with primitives.

    The finger body, and on top of it the raised pad, either flat or with
    the V cut into it.
    """
    lo, hi = stl_bbox(tris)
    # The inner face is whichever x bound is further from the link origin; the
    # left and right tips are mirror images, so derive the sense rather than
    # hard-coding it.
    inner, outer = (lo[0], hi[0]) if abs(lo[0]) > abs(hi[0]) else (hi[0], lo[0])
    out = 1.0 if outer > inner else -1.0
    (pad_y0, pad_y1), (pad_z0, pad_z1) = real_pad_face(tris, inner)

    y_c = (pad_y0 + pad_y1) / 2.0
    width = pad_y1 - pad_y0
    back = inner + out * PAD_THICKNESS      # where the pad meets the body
    root = inner + out * GROOVE_DEPTH       # deepest point of the V

    for col in list(link.findall('collision')):
        link.remove(col)

    # Finger body: everything behind the pad, over the fingertip's full
    # footprint. Stops at the pad's back face so it never reaches the contact
    # plane where the real fingertip is relieved.
    box(link, 'finger_body',
        (abs(outer - back), hi[1] - lo[1], hi[2] - lo[2]),
        ((back + outer) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0))

    if PAD_MODE == 'flat':
        box(link, 'pad', (PAD_THICKNESS, width, pad_z1 - pad_z0),
            ((inner + back) / 2.0, y_c, (pad_z0 + pad_z1) / 2.0))
        return

    flank_len = math.hypot(GROOVE_HALF_HEIGHT, GROOVE_DEPTH)
    theta = math.atan2(GROOVE_DEPTH, GROOVE_HALF_HEIGHT)
    ct, st = math.cos(theta), math.sin(theta)
    half_t = FLANK_THICKNESS / 2.0
    lip = GROOVE_CENTRE_Z - GROOVE_HALF_HEIGHT

    # Flat part of the pad, below the groove, at the original contact plane.
    box(link, 'pad_flat', (PAD_THICKNESS, width, lip - pad_z0),
        ((inner + back) / 2.0, y_c, (pad_z0 + lip) / 2.0))

    # Lower flank: from the wing at the pad plane, out to the groove root.
    # The box is rotated so its inner face lies on the flank, then pushed
    # half its thickness outward so the material sits behind that face.
    box(link, 'pad_flank_lower', (FLANK_THICKNESS, width, flank_len),
        ((inner + root) / 2.0 + out * ct * half_t,
         y_c,
         GROOVE_CENTRE_Z - GROOVE_HALF_HEIGHT / 2.0 - st * half_t),
        pitch=out * theta)

    # Upper flank: mirror of the lower one about the groove centre.
    box(link, 'pad_flank_upper', (FLANK_THICKNESS, width, flank_len),
        ((inner + root) / 2.0 + out * ct * half_t,
         y_c,
         GROOVE_CENTRE_Z + GROOVE_HALF_HEIGHT / 2.0 + st * half_t),
        pitch=-out * theta)

    # Fills the pad behind the groove, from the root back to the body.
    box(link, 'pad_backing', (abs(back - root), width, pad_z1 - lip),
        ((root + back) / 2.0, y_c, (lip + pad_z1) / 2.0))


def main():
    urdf = subprocess.run(['xacro'] + sys.argv[1:],
                          capture_output=True, text=True, check=True).stdout
    root = ET.fromstring(urdf)

    done = 0
    for link in root.findall('link'):
        if not is_fingertip(link.get('name')):
            continue
        mesh = link.find('collision/geometry/mesh')
        if mesh is None:
            continue
        if PAD_MODE == 'groove':
            # Same mesh, minus the twelve pad-face triangles, plus a V.
            side = 'left' if 'left' in link.get('name') else 'right'
            mesh.set('filename', GROOVED_MESH.format(side=side))
        elif PAD_MODE == 'hook':
            side = 'left' if 'left' in link.get('name') else 'right'
            hook_fingertip(link, stl_tris(
                mesh.get('filename').replace('file://', '')), side)
        elif PAD_MODE in ('boxes', 'flat'):
            groove_pad(link, stl_tris(mesh.get('filename').replace('file://', '')))
        elif PAD_MODE == 'wide':
            widen_pad(link, stl_tris(mesh.get('filename').replace('file://', '')))
        elif PAD_MODE != 'mesh':
            print(f'unknown BARTENDER_PAD_MODE {PAD_MODE!r}', file=sys.stderr)
            return 1
        gz = ET.SubElement(root, 'gazebo')
        gz.set('reference', link.get('name'))
        ET.SubElement(gz, 'mu1').text = str(PAD_MU)
        ET.SubElement(gz, 'mu2').text = str(PAD_MU)
        done += 1

    # Two per gripper, and the count is not hard-coded to two grippers: the
    # check that matters is that no fingertip was MISSED, which would leave
    # one arm quietly holding bottles on stock pads at stock friction. An odd
    # count means a gripper came through half-matched, which is a bug in the
    # suffix match rather than in the description.
    if done == 0 or done % len(TIPS) != 0:
        print(f'expected a whole number of grippers worth of fingertips '
              f'({len(TIPS)} each), did {done}', file=sys.stderr)
        return 1

    sys.stdout.write(ET.tostring(root, encoding='unicode'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
