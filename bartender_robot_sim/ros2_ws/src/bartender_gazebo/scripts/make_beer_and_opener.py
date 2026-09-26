#!/usr/bin/env python3
"""Generate the beer bottle, its crown cap, the opener, and the opener holster.

These four are one design, not four models, which is why they are generated
from one file: the cap is sized to the bottle's crown finish, the opener's
bell is sized to the cap, and the holster's post is sized to the bell. Change
any one number by hand in the SDF and the other three quietly stop matching.
bartender_open imports the numbers from here rather than restating them.

WHAT "OPENING" MEANS HERE, AND WHAT IS REAL
-------------------------------------------
Be clear about this, because it is the one place where the simulation is not
doing the physics.

A real crown cap comes off because a lip catches under a 0.25mm-thick skirt
and levers it open against the crimp. Nothing at this fidelity reproduces
that: DART collides a dynamic body's mesh as its CONVEX HULL, contact is
resolved on a 1ms step, and an interference fit measured in tenths of a
millimetre between two bodies the solver is already struggling with is how you
get an explosion, not a pop. See PAD_MODE 'groove' in
render_bartender_urdf.py for the last time this project tried to make a
concave feature do mechanical work.

So the cap is a SEPARATE MODEL held to the bottle by a DetachableJoint, and
the release is commanded on a topic.

What is NOT scripted is WHETHER the release happens. bartender_open only
sends the detach once it has measured, from the simulator's own state, that

  - the opener is seated over the cap (the bell has actually descended over
    it, not stopped short or missed sideways), and
  - the beer has not moved while being pushed on -- i.e. the other arm really
    did hold it.

Push on a beer nobody is holding and it is driven down into its stand or
knocked off it, the seating check fails, and the cap stays on. That coupling
is the point of the exercise and it is measured, not asserted.

WHY THE BELL, AND WHY NO PRY LIP
--------------------------------
The opener is a bell -- a socket that drops over the cap -- with a flat crown
plate across the top of it and a shaft above that for the gripper.

The bell does three jobs. It self-centres: 2.5mm of radial clearance means
the last few millimetres of the descent are guided by the wall rather than by
the arm's ~3mm Cartesian tracking error. It gives the descent a hard stop
(the crown plate on the cap top) so "pressed home" is a definite event. And
it makes the seating measurable: either the bell rim is below the cap top or
it is not.

It has no pry lip, deliberately. A lip would have to reach INSIDE the cap's
outer diameter to catch the skirt, which is exactly the interference the
first section says does not survive here -- and since the release is
commanded anyway, the lip would be decoration that jitters.

The bell is open at the bottom, so after the detach the freed cap drops out
of it under gravity as the opener is lifted away. That part is real physics,
and it is what bartender_open checks to call the bottle open.

THE BELL IS A RING OF BOXES
---------------------------
Same reason the bottle stands are: a ring is concave and its convex hull is a
solid disc. A bell modelled as one mesh would not fit over the cap at all, it
would sit on top of it.

FRAMES
------
Every model's link origin is at the centre of its footprint, at the height it
rests on the counter, so a model pose is just the station's (x, y, counter).
The opener is the exception and says so at its own definition.
"""
import argparse
import math
import os

# ---------------------------------------------------------------- beer bottle
#
# Measured off models/beer_16oz_bottle.obj (2882 verts, Z-up, centimetres,
# origin at the base centre) by taking the maximum radius in each ring of
# vertices. The profile is a real 473ml / 16oz longneck with a proper crown
# finish, which is what makes it usable here at all -- a bottle whose profile
# only ever narrows going up has nothing for a cap to sit on:
#
#     z 0.000..0.130   body, 64.4mm
#     z 0.130..0.184   shoulder, 64.4 -> 35.8mm
#     z 0.184..0.242   neck, 35.8 -> 23.3mm
#     z 0.242..0.2581  crown finish: a transfer bead at 27.3mm (z 0.244) and
#                      the sealing lip at 27.1mm (z 0.253..0.257)
#
# The mesh is the visual; the collisions below are the usual stack of
# cylinders. Grasp is on the BODY, which is a plain 64.4mm cylinder over
# 130mm -- no waist to seat in, unlike the cola, and none needed: the pads
# open to 85mm so a 64.4mm body leaves 10mm a side.
BEER_MESH_SCALE = 0.01          # the OBJ is in centimetres
BEER_HEIGHT = 0.2581            # top of the crown finish, above the base
BEER_BODY_RADIUS = 0.0322

# WHERE THE BOTTLE IS HELD, AND WHY IT IS NOT THE BODY
# ----------------------------------------------------
# It is held by the NECK, at the straight band between 0.170 and 0.200, and
# that is forced.
#
# The body is a 64.4mm smooth cylinder over its whole lower half, which looks
# like the obvious place to grip and is the one place this gripper cannot.
# Two flat pads on a smooth cylinder are a badly conditioned contact: the
# bottle is extruded from between them rather than stopping them, and it goes
# sideways. That is not a guess, it is this project's most expensive lesson
# twice over -- pour_action_server's COLA notes record clamp 0.30, 0.35 and
# 0.42 all losing the 67.2mm bulge, and the first run of the opening sequence
# gripped this bottle's body at 64.4mm and threw it 3.8 metres off the
# counter.
#
# What made the cola work was gripping its WAIST, where the wider sections
# above and below are stops the closed pads cannot pass, so slip is bounded
# by geometry instead of by friction. This bottle has no waist -- but it has
# a neck, and a neck is the same feature seen from the other side:
#
#     0.152..0.170   shoulder, out to 51.0mm     <- the stop
#     0.170..0.200   neck, 38.7mm and straight   <- the pads go here
#     0.200..0.242   neck tapering to 23.3mm
#
# a 6.2mm radial step, right under a 30mm straight band. The pads are 31mm
# and so cannot sit inside it; they overhang the TOP, where the bottle tapers
# away and the overhang touches nothing, and clear the shoulder below by
# 3.5mm. See BEER_GRASP_HEIGHT in bartender_open/layout.py, which is what
# places them, and note that the band being narrower than the pad is why that
# number is not simply the middle of this band -- and that moving it to make
# room for a wider pad was measured worse.
#
# And the step is the right way round for what happens next: the whole
# point of this sequence is that something pushes DOWN on this bottle, down
# is into the widening shoulder, and a wedge driven into closing pads grips
# harder rather than slipping.
#
# It also hangs well. The centre of mass is at 0.105, 80mm BELOW the pads, so
# the bottle is a pendulum and self-centres instead of pitching.
BEER_GRIP_BAND = (0.170, 0.200)

# Collision bands, as (z0, z1). Radii are NOT written here: they are measured
# from the mesh at generation time, taking each band's widest point, so the
# collision can never quietly stop matching the bottle it is drawn as. The
# edges are hand-placed at the profile's own features -- the base of the
# shoulder, the ends of the straight neck, the crown finish.
#
# Widest point, rather than mean: it makes every step slightly proud, which
# is the safe direction for a feature whose entire job is to be a stop.
BEER_BANDS = (
    (0.000, 0.130),     # body
    (0.130, 0.152),     # shoulder, still full width
    (0.152, 0.170),     # shoulder tapering in -- the stop under the grip
    BEER_GRIP_BAND,     # straight neck: where the pads close
    (0.200, 0.222),     # neck tapering
    (0.222, 0.242),     # ...and further
    (0.242, 0.2581),    # crown finish: bead, groove, sealing lip
)

# 473ml of beer plus a 473ml bottle's worth of glass, which for a returnable
# longneck is about 230g. Left at the honest figure rather than inflated the
# way the whiskey and cola were: those were made heavy to survive being
# knocked during place-and-release, and this bottle is never released
# free-standing -- it goes back into a well (beer_stand) and it is held by the
# other arm for the only part of the cycle where anything pushes on it.
BEER_MASS = 0.70
BEER_COM_Z = 0.105

# --------------------------------------------------------------------- the cap
#
# A crown cap is 32.1mm across and 6.3mm deep with the skirt crimped down over
# the bead. Modelled as a plain disc SITTING ON the lip rather than as a skirt
# wrapping over it, and that is a choice worth defending: the skirt version
# means the cap's collision and the bottle's crown finish occupy the same
# space. Two bodies joined by a DetachableJoint are one skeleton while they
# are joined, so that is survivable -- but the instant the joint is released
# they are two bodies interpenetrating by 4mm, and the solver's answer to that
# is to fire the cap across the room. A disc on the lip separates cleanly.
CAP_RADIUS = 0.0161
CAP_HEIGHT = 0.0065
CAP_MASS = 0.0022               # a real crown cap is a bit over 2 grams

# --------------------------------------------------------------------- opener
#
# ORIGIN. Unlike everything else here the opener's link origin is at the BELL
# RIM -- the bottom of the socket, the face that goes over the cap -- and not
# at the bottom of the model, because there is nothing else useful at the
# bottom of the model: the rim IS the bottom. Everything is at positive local
# z. So placing the opener at (x, y, counter) stands it on its rim.
BELL_CLEARANCE = 0.0025         # radial slop between cap and bell wall
BELL_WALL = 0.005
BELL_BORE_H = 0.016             # the straight part, which guides and holds

# A chamfered mouth at the rim, stepped outward, exactly like the lead-in on
# the bottle stands and on the holster post -- and for exactly the same
# reason, which was learned here the same way.
#
# A straight bore clears the cap by BELL_CLEARANCE, so it has to be brought
# down within 2.5mm of the cap's centre or its rim lands ON the cap instead
# of over it. Measured, arm B put it 5.4mm off: not much, and far better than
# the 7.3mm before the aim was taken from the cap's real pose rather than its
# station, but still more than 2.5mm, and the bell just sat on top with the
# seating reading -1.6mm of a possible 16.
#
# Stepping the mouth out by BELL_LEAD turns that into a funnel: an error up
# to BELL_CLEARANCE + BELL_LEAD is guided in over the chamfer instead of
# parking on the rim.
BELL_LEAD = 0.004
BELL_LEAD_H = 0.005
BELL_SEGMENTS = 16
PLATE_THICKNESS = 0.006         # bears on the cap top; this is what pushes

# 24mm square. Square on purpose: the whiskey bottle's flat faces stall the
# knuckle within 0.2mm of first touch where the round cola needs ~6mm, so a
# flat-sided shaft is the easiest thing in this scene for these pads to hold,
# and the opener is the one object that gets pushed against while gripped.
# 24mm also leaves the pads 30mm a side of travel from their 85mm open, so the
# approach clearance is not tight.
SHAFT = 0.024
SHAFT_LENGTH = 0.100

OPENER_MASS = 0.12              # cast stainless, about right for the volume
OPENER_MU = 0.8                 # steel; the pads are 1.8

# ------------------------------------------------------------------- holster
#
# A post the bell drops over, not a well the opener stands in. Both work; the
# post is better here because the bell is already a socket, so the post reuses
# the alignment feature the opener has instead of adding a second one, and
# because putting the opener back is then the same downward move as putting it
# on the cap.
#
# The top course is narrower, for the same reason the bottle stands have a
# lead-in: a straight post the bell clears by 2.5mm would have to be found to
# within 2.5mm, and Cartesian moves in this system land within about 3mm.
# Stepping the top in by 2.6mm turns that into a 5.1mm capture.
POST_LEAD = 0.0026
POST_HEIGHT = 0.010
POST_LEAD_HEIGHT = 0.004

# WHAT THE OPENER ACTUALLY RESTS ON. Its bell RIM, flat on the counter, with
# the post standing inside the bell as a locator and touching nothing.
#
# That is why the post is 14mm against the bell's 16mm depth. The first cut
# made them both 16mm, which stood the opener on two things at once -- the
# rim on the counter and the crown plate on the post top -- and whichever of
# them won was down to rounding. It came to rest on its rim (measured in sim:
# model z exactly 0.900, the counter), so the post was shortened to make that
# the only answer. A 47mm annulus flat on the counter is a far better
# footprint than balancing on a 32mm post anyway.
#
# The consequence for everything else is that the opener's origin at rest is
# at COUNTER LEVEL, not at the post top. bartender_open's OPENER_REST_RIM_Z
# says so.

CAP_MU = 0.9                    # steel cap on glass, same as the bottles
HOLSTER_MU = 1.2                # matches the bottle stands

# Where the DetachableJoint in beer_bottle listens. The world does not name
# this; the plugin tag generated below does, and bartender_open publishes to
# it through a ros_gz bridge.
DETACH_TOPIC = '/beer/cap/detach'


def _mesh_path():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(
        here, *([os.pardir] * 4),
        'models', 'beer_bottle', 'meshes', 'beer_bottle.obj'))


def beer_profile(path=None):
    """Return (z, radius) for every vertex of the beer mesh, in metres."""
    out = []
    with open(path or _mesh_path(), errors='ignore') as fh:
        for line in fh:
            if not line.startswith('v '):
                continue
            x, y, z = (float(v) * BEER_MESH_SCALE for v in line.split()[1:4])
            out.append((z, math.hypot(x, y)))
    if not out:
        raise SystemExit('no vertices in the beer mesh')
    return out


def beer_bands(path=None):
    """Measure (z0, z1, widest radius) for each collision band off the mesh."""
    profile = beer_profile(path)
    bands = []
    for z0, z1 in BEER_BANDS:
        radii = [r for z, r in profile if z0 - 1e-9 <= z <= z1 + 1e-9]
        if not radii:
            raise SystemExit(f'no mesh vertices between z {z0} and {z1}')
        bands.append((z0, z1, max(radii)))
    return bands


def beer_grip_radius():
    """Return the radius the pads actually close onto."""
    z0, z1 = BEER_GRIP_BAND
    return dict(((a, b), r) for a, b, r in beer_bands())[(z0, z1)]


def bell_inner_radius():
    return CAP_RADIUS + BELL_CLEARANCE


def bell_height():
    """Return the full seating depth, rim to the crown plate's underside."""
    return BELL_LEAD_H + BELL_BORE_H


def bell_capture():
    """Return the lateral error the mouth will still funnel in."""
    return BELL_CLEARANCE + BELL_LEAD


def bell_outer_radius():
    return bell_inner_radius() + BELL_WALL


def opener_height():
    """Return the opener's overall height, rim to the top of the shaft."""
    return bell_height() + PLATE_THICKNESS + SHAFT_LENGTH


def seated_rim_z(cap_top_z):
    """Return the rim's world z with the opener pressed fully home.

    With the crown plate flat on the cap top, the rim is one bell-height
    below it. This is the number bartender_open compares the measured rim
    against to decide the opener is seated rather than resting on the rim of
    the cap or stopped in the air above it.
    """
    return cap_top_z - bell_height()


def shaft_grip_z():
    """Return the local z to put the gripper's pad centre at.

    Middle of the shaft: it is the only part of the opener the pads can close
    on, and centring keeps the 30mm-wide pads clear of both the crown plate
    below and the shaft's top end above.
    """
    return bell_height() + PLATE_THICKNESS + SHAFT_LENGTH / 2.0


def post_top():
    return POST_HEIGHT + POST_LEAD_HEIGHT


def bell_courses():
    """Return the bell's two stacked rings: (inner radius, z of base, height).

    The chamfered mouth is the LOWER one, because the rim is the bottom and
    the cap comes in from below.
    """
    return [(bell_inner_radius() + BELL_LEAD, 0.0, BELL_LEAD_H),
            (bell_inner_radius(), BELL_LEAD_H, BELL_BORE_H)]


def bell_segment_boxes():
    """Lay out (xyz, yaw, size) for each box in the bell wall, both courses.

    Laid out exactly like make_bottle_stands.py's wall: inner face tangent to
    the bore circle so the polygon's inradius is the quoted clearance, length
    from the outer tangent circle so neighbours overlap at the corners rather
    than leaving 16 gaps.
    """
    for inner_r, z0, height in bell_courses():
        length = 2.0 * (inner_r + BELL_WALL) * math.tan(math.pi / BELL_SEGMENTS)
        for i in range(BELL_SEGMENTS):
            angle = 2.0 * math.pi * i / BELL_SEGMENTS
            r = inner_r + BELL_WALL / 2.0
            yield ((r * math.cos(angle), r * math.sin(angle), z0 + height / 2.0),
                   angle, (BELL_WALL, length, height))


GENERATED = ('<!-- GENERATED by '
             'ros2_ws/src/bartender_gazebo/scripts/make_beer_and_opener.py.\n'
             '     Edit that script and re-run it; edits here are '
             'overwritten. -->')


def _friction(mu):
    return (f'<surface><friction><ode><mu>{mu}</mu>'
            f'<mu2>{mu}</mu2></ode></friction></surface>')


def _cyl(radius, length, z, mu, name):
    return f"""
      <collision name="{name}">
        <pose>0 0 {z:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{radius:.6f}</radius>
          <length>{length:.6f}</length></cylinder></geometry>
        {_friction(mu)}
      </collision>"""


def beer_sdf():
    # Solid-cylinder proxy on the body radius and the full height, the same
    # approximation the other two bottles use. Recompute both if the mass or
    # the bottle changes.
    ixx = BEER_MASS * (3.0 * BEER_BODY_RADIUS ** 2 + BEER_HEIGHT ** 2) / 12.0
    izz = BEER_MASS * BEER_BODY_RADIUS ** 2 / 2.0
    return f"""<?xml version="1.0" ?>
{GENERATED}
<sdf version="1.9">
  <model name="beer_bottle">
    <!-- 473ml / 16oz longneck with a crown finish, origin at the centre of
         the base like the other bottles, so the model pose is the station
         pose at counter height.

         The CAP IS NOT PART OF THIS MODEL. It is a separate model held on by
         the DetachableJoint below, because that is the only way to take it
         off again. The joint is released by publishing on
         {DETACH_TOPIC}. -->
    <link name="beer_link">
      <inertial>
        <pose>0 0 {BEER_COM_Z:.4f} 0 0 0</pose>
        <mass>{BEER_MASS}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx>
          <iyy>{ixx:.6f}</iyy>
          <izz>{izz:.6f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <!-- Same damping the cola needed: a cylinder standing on a flat counter
           gets a narrow contact manifold and will spin on it indefinitely. -->
      <velocity_decay>
        <linear>0.05</linear>
        <angular>0.30</angular>
      </velocity_decay>

      <visual name="beer_visual">
        <geometry>
          <mesh>
            <uri>meshes/beer_bottle.obj</uri>
            <scale>{BEER_MESH_SCALE} {BEER_MESH_SCALE} {BEER_MESH_SCALE}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.05 0.03 0.01 1</ambient>
          <diffuse>0.35 0.19 0.05 1</diffuse>
          <specular>0.6 0.6 0.6 1</specular>
        </material>
      </visual>
{''.join(_cyl(r, z1 - z0, (z0 + z1) / 2.0, 0.9, f'beer_band_{i}_collision')
             for i, (z0, z1, r) in enumerate(beer_bands()))}
    </link>

    <!-- Holds beer_cap on. child_model is a world-level model NAME, so the
         world must include the cap under exactly this name; the two are only
         a pair by that string. Detaching is one-way and one-shot: this
         plugin has no re-attach in Fortress, so a cap that has come off stays
         off until the world is reloaded. -->
    <plugin filename="ignition-gazebo-detachable-joint-system"
            name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>beer_link</parent_link>
      <child_model>beer_cap</child_model>
      <child_link>cap_link</child_link>
      <detach_topic>{DETACH_TOPIC}</detach_topic>
    </plugin>
  </model>
</sdf>
"""


def cap_sdf():
    ixx = CAP_MASS * (3.0 * CAP_RADIUS ** 2 + CAP_HEIGHT ** 2) / 12.0
    izz = CAP_MASS * CAP_RADIUS ** 2 / 2.0
    return f"""<?xml version="1.0" ?>
{GENERATED}
<sdf version="1.9">
  <model name="beer_cap">
    <!-- The crown cap, as its own model so a DetachableJoint can let go of
         it. Origin at the centre of its UNDERSIDE, so the world pose is the
         beer's station (x, y) at counter + {BEER_HEIGHT:.4f} - the bottle's
         lip. Anywhere else and it starts the world interpenetrating the
         bottle it is joined to.

         Two grams. It is deliberately not rounded up: once detached this is
         a free body sharing a contact with the opener bell and the bottle
         lip, and a heavy one pushes them around. -->
    <link name="cap_link">
      <inertial>
        <pose>0 0 {CAP_HEIGHT / 2.0:.6f} 0 0 0</pose>
        <mass>{CAP_MASS}</mass>
        <inertia>
          <ixx>{ixx:.9f}</ixx>
          <iyy>{ixx:.9f}</iyy>
          <izz>{izz:.9f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <visual name="cap_visual">
        <pose>0 0 {CAP_HEIGHT / 2.0:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{CAP_RADIUS:.6f}</radius>
          <length>{CAP_HEIGHT:.6f}</length></cylinder></geometry>
        <material>
          <ambient>0.20 0.02 0.02 1</ambient>
          <diffuse>0.70 0.08 0.06 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
{_cyl(CAP_RADIUS, CAP_HEIGHT, CAP_HEIGHT / 2.0, CAP_MU, 'cap_collision')}
    </link>
  </model>
</sdf>
"""


def opener_sdf():
    parts = []
    for i, (xyz, yaw, size) in enumerate(bell_segment_boxes()):
        pose = '{:.6f} {:.6f} {:.6f} 0 0 {:.6f}'.format(*xyz, yaw)
        dims = '{:.6f} {:.6f} {:.6f}'.format(*size)
        parts.append(f"""
      <visual name="bell_{i}">
        <pose>{pose}</pose>
        <geometry><box><size>{dims}</size></box></geometry>
        <material>
          <ambient>0.14 0.14 0.15 1</ambient>
          <diffuse>0.55 0.56 0.60 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular>
        </material>
      </visual>
      <collision name="bell_collision_{i}">
        <pose>{pose}</pose>
        <geometry><box><size>{dims}</size></box></geometry>
        {_friction(OPENER_MU)}
      </collision>""")

    plate_z = bell_height() + PLATE_THICKNESS / 2.0
    bore_d = bell_inner_radius() * 2.0
    mouth_d = (bell_inner_radius() + BELL_LEAD) * 2.0
    plate_top = bell_height() + PLATE_THICKNESS
    cap_d = CAP_RADIUS * 2.0
    side = BELL_CLEARANCE * 1000.0
    mouth_side = (BELL_CLEARANCE + BELL_LEAD) * 1000.0
    capture = bell_capture() * 1000.0
    bell_h = bell_height()
    shaft_mm = SHAFT * 1000.0
    shaft_z = shaft_grip_z()
    metal = """<material>
          <ambient>0.14 0.14 0.15 1</ambient>
          <diffuse>0.55 0.56 0.60 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular>
        </material>"""

    # Inertia: the shaft is most of the length and nearly all of the moment,
    # so take the whole thing as a slender box SHAFT x SHAFT x opener_height
    # about its own centre. It over-estimates izz a little (the bell is
    # further out than the shaft) and that is the harmless direction.
    h = opener_height()
    ixx = OPENER_MASS * (SHAFT ** 2 + h ** 2) / 12.0
    izz = OPENER_MASS * (SHAFT ** 2 + SHAFT ** 2) / 12.0
    return f"""<?xml version="1.0" ?>
{GENERATED}
<sdf version="1.9">
  <model name="bottle_opener">
    <!-- Bell-and-shaft crown opener. ORIGIN IS AT THE BELL RIM, the face
         that goes over the cap, with everything above it at positive z:

           z 0.000..{BELL_LEAD_H:.3f}   chamfered mouth, {mouth_d:.4f} across
           z {BELL_LEAD_H:.3f}..{bell_h:.3f}   bell bore, {bore_d:.4f} across
           z {bell_h:.3f}..{plate_top:.3f}   crown plate - it pushes
           z {plate_top:.3f}..{opener_height():.3f}   {shaft_mm:.0f}mm square shaft

         The bore clears the {cap_d:.4f} cap by {side:.1f}mm a side and the
         mouth by {mouth_side:.1f}mm, so an approach up to {capture:.1f}mm off
         centre is funnelled in rather than parked on the cap, and the last
         few millimetres of
         the descent are guided by the wall rather than by the arm.

         No pry lip, on purpose - see the module docstring of the generator.
         The bell is a socket, and the bottom is open so the freed cap falls
         out of it when the opener is lifted clear. -->
    <link name="opener_link">
      <inertial>
        <pose>0 0 {h / 2.0:.6f} 0 0 0</pose>
        <mass>{OPENER_MASS}</mass>
        <inertia>
          <ixx>{ixx:.8f}</ixx>
          <iyy>{ixx:.8f}</iyy>
          <izz>{izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
{''.join(parts)}

      <visual name="crown_plate">
        <pose>0 0 {plate_z:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{bell_outer_radius():.6f}</radius>
          <length>{PLATE_THICKNESS:.6f}</length></cylinder></geometry>
        {metal}
      </visual>
{_cyl(bell_outer_radius(), PLATE_THICKNESS, plate_z, OPENER_MU, 'crown_plate_collision')}

      <visual name="shaft">
        <pose>0 0 {shaft_z:.6f} 0 0 0</pose>
        <geometry><box><size>{SHAFT:.6f} {SHAFT:.6f}
          {SHAFT_LENGTH:.6f}</size></box></geometry>
        {metal}
      </visual>
      <collision name="shaft_collision">
        <pose>0 0 {shaft_z:.6f} 0 0 0</pose>
        <geometry><box><size>{SHAFT:.6f} {SHAFT:.6f}
          {SHAFT_LENGTH:.6f}</size></box></geometry>
        {_friction(OPENER_MU)}
      </collision>
    </link>
  </model>
</sdf>
"""


def holster_sdf():
    # The post IS a cap: the bell already has a bore sized to drop over a
    # crown cap with BELL_CLEARANCE to spare, so making the post cap-sized
    # means putting the opener away is mechanically the same move as putting
    # it on the bottle, with the same tolerance.
    post_r = CAP_RADIUS
    lead_r = post_r - POST_LEAD
    top_capture = (POST_LEAD + BELL_CLEARANCE) * 1000.0
    bottom_capture = BELL_CLEARANCE * 1000.0
    shortfall = (bell_height() - post_top()) * 1000.0
    return f"""<?xml version="1.0" ?>
{GENERATED}
<sdf version="1.9">
  <model name="opener_holster">
    <!-- Where the opener lives when it is not in a gripper. A stepped post
         the bell drops over, origin at counter level:

           z 0.000..{POST_HEIGHT:.3f}   post, r {post_r:.4f}
           z {POST_HEIGHT:.3f}..{post_top():.3f}   lead-in, r {lead_r:.4f}

         Capture is {top_capture:.1f}mm at the top, narrowing to
         {bottom_capture:.1f}mm at the bottom, against Cartesian moves that
         land within about 3mm.

         Static, and it is a post rather than a well so the opener's own bell
         does the locating. The post is {shortfall:.0f}mm shorter than the
         bell is deep, so the opener comes to rest on its RIM, on the
         counter, with the post inside it touching nothing. -->
    <static>true</static>
    <link name="holster_link">
{_cyl(post_r, POST_HEIGHT, POST_HEIGHT / 2.0, HOLSTER_MU, 'post_collision')}
{_cyl(lead_r, POST_LEAD_HEIGHT, POST_HEIGHT + POST_LEAD_HEIGHT / 2.0,
      HOLSTER_MU, 'post_lead_collision')}
      <visual name="post">
        <pose>0 0 {POST_HEIGHT / 2.0:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{post_r:.6f}</radius>
          <length>{POST_HEIGHT:.6f}</length></cylinder></geometry>
        <material>
          <ambient>0.10 0.10 0.12 1</ambient>
          <diffuse>0.16 0.16 0.19 1</diffuse>
        </material>
      </visual>
      <visual name="post_lead">
        <pose>0 0 {POST_HEIGHT + POST_LEAD_HEIGHT / 2.0:.6f} 0 0 0</pose>
        <geometry><cylinder><radius>{lead_r:.6f}</radius>
          <length>{POST_LEAD_HEIGHT:.6f}</length></cylinder></geometry>
        <material>
          <ambient>0.10 0.10 0.12 1</ambient>
          <diffuse>0.16 0.16 0.19 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


MODELS = {
    'beer_bottle': (beer_sdf, '473ml longneck beer bottle with a crown finish'),
    'beer_cap': (cap_sdf, 'crown cap, detachable from beer_bottle'),
    'bottle_opener': (opener_sdf, 'bell-and-shaft crown opener'),
    'opener_holster': (holster_sdf, 'stepped post the opener stands on'),
}


def config(name, note):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{note}. Generated by make_beer_and_opener.py.</description>
</model>
"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.normpath(
        os.path.join(here, *([os.pardir] * 4), 'models'))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default=default_out,
                    help='models/ directory to write into')
    args, _ = ap.parse_known_args()

    for name, (builder, note) in MODELS.items():
        path = os.path.join(args.out, name)
        os.makedirs(path, exist_ok=True)
        with open(os.path.join(path, 'model.sdf'), 'w') as fh:
            fh.write(builder())
        with open(os.path.join(path, 'model.config'), 'w') as fh:
            fh.write(config(name, note))
        print(f'{path}')

    print(f'  beer height        {BEER_HEIGHT:.4f}  body r {BEER_BODY_RADIUS:.4f}')
    for z0, z1, r in beer_bands():
        mark = '  <- pads close here' if (z0, z1) == BEER_GRIP_BAND else ''
        print(f'    band {z0:.3f}-{z1:.3f}  r {r:.4f} '
              f'(D {r * 2000:5.1f}mm){mark}')
    print(f'  cap                r {CAP_RADIUS:.4f} h {CAP_HEIGHT:.4f}, '
          f'top at {BEER_HEIGHT + CAP_HEIGHT:.4f} above the base')
    print(f'  bell bore          r {bell_inner_radius():.4f} '
          f'({BELL_CLEARANCE * 1000:.1f}mm over the cap), depth {bell_height():.4f}, '
          f'capture {bell_capture() * 1000:.1f}mm')
    print(f'  opener height      {opener_height():.4f}, '
          f'grip at local z {shaft_grip_z():.4f}')
    print(f'  holster post       r {bell_inner_radius() - BELL_CLEARANCE:.4f}, '
          f'top at {post_top():.4f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
