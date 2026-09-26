"""Where everything is, and how the two arms get at it.

No ROS in this module, on purpose: every number the opening sequence depends
on is decided here and can be checked by a test that does not need a
simulator, a build, or a running move_group. The node in
open_action_server.py does the talking; this does the thinking.

THE FRAME PROBLEM
-----------------
There are three frames and it is worth being blunt about them, because mixing
them up is silent and produces a plan that just misses.

  world        Gazebo's. The world SDF poses everything in it. Not a TF
               frame anything in ROS plans against.
  base_link    arm A's base. bartender_pour plans everything in it. It is at
               world (-0.45, -0.375, 0.9) -- that is the ros_gz `create`
               spawn pose in bartender_gazebo/launch/sim.launch.py, not a
               coincidence and not written down anywhere else.
  b_base_link  arm B's base, at world (-0.45, 0.375, 0.9).

The two arms differ by a PURE TRANSLATION of 0.75 along arm A's +y: same
height, same yaw, side by side on the same bar top facing the same way. So
converting between them is an offset, arm B's poses are arm A's with y
mirrored about the bar's centreline, and b_home is just home. That is worth
the paragraph because it used not to be true -- arm B stood on a pedestal
beside a smaller counter, yawed -90 degrees to face back across it, and
every pose for it had to be worked out separately.

Both arm bases sit at world z = 0.9, which is the counter top. So a z in
either arm's frame is "height above the counter", the two agree, and the
bottle geometry (measured up from a bottle's base, which rests on the
counter) drops straight in. That is a convenience the layout was chosen to
keep, not a given.

THE BAR TOP
-----------
1.5 x 2.0, centred at world (-0.10, 0), top at 0.9. Three rows, and every
station is on one of them:

  the arms        x = -0.45. A at y = -0.40, B at y = +0.40.
  the bottle line x = 0.07, so 0.52 in front of both arms. Nine slots at a
                  0.20 pitch, y = -0.80 to +0.80. whiskey, cola and beer
                  occupy three, two are pour lanes that stay empty, and the
                  remaining four are free: this is where another bottle
                  goes, and putting one there should cost a slot and a
                  station name, not a re-survey.
  serving strip   x = 0.22. The glass on arm A's centreline, the opener's
                  holster on arm B's, 0.67 dead ahead of each.

WHY A LINE, AND WHY THAT WAY ROUND
----------------------------------
The line runs ACROSS each arm's approach rather than along it. bartender_pour
grasps from the side, running the gripper in along the arm's own +x, so every
slot at a common x has its own clear lane in front of it and no bottle ever
stands behind another. A row of bottles laid out along the approach direction
would put the far ones behind the near ones, which is the one arrangement a
side grasp cannot use.

It also makes the stations cheap to reason about: each arm's reach to a slot
is hypot(0.52, |slot_y - arm_y|), which is 0.52 for a slot straight ahead and
0.656 at the far end of what either arm can service.

The beer takes the middle slot because it is the one station that has to
satisfy two arms at once -- one holds the bottle while the other pushes down
on it -- and the middle of a bar with an arm at each side is equidistant by
construction rather than by search.

WHAT SETS THE TWO ROW DEPTHS
----------------------------
Not taste. A UR5e's usable band here runs from about 0.29 of flange radius
(closer and the gripper folds back over the arm's own shoulder) to about
0.68 of station radius, and both rows plus their stand-offs have to fit in
that 0.39m.

  The bottle line is at 0.52 because bartender_pour approaches a side grasp
  from GRIP_AHEAD_OF_TOOL0 + APPROACH_BACKOFF = 0.245 back along the arm's
  +x. That puts the approach flange at 0.275, at a radius of 0.340 with the
  slot's y offset -- the same 0.340 the old layout's whiskey approach sat
  at, which is the one value here known to work.

  The serving strip is at 0.67 because the opener has to stay inside 0.68
  from arm B: not for the pick, which is comfortable, but for the cruise
  over the holster at OPENER_TRANSIT_Z, where the flange is 0.713 from the
  base and the limit is 0.75.

That leaves 0.15 between the rows, and the reason that is enough is the
lanes: the stations on the strip do not line up with any bottle.

These are duplicated in bartender_gazebo/worlds/bar_world.sdf, which is what
actually places the models, and in bartender_pour, which states its own three
in arm A's frame. test_layout.py compares all of them against this file.
"""
import math

# ---------------------------------------------------------------- the frames

# Arm A's base in world coordinates: the spawn pose in sim.launch.py.
ARM_A_ORIGIN = (-0.45, -0.40, 0.9)
ARM_A_YAW = 0.0

# Arm B's base in world coordinates, facing back down the bar at arm A. The
# xacro states the same point relative to arm A, as ARM_B_IN_A below, because
# that is how it is attached; these two have to describe the same place.
#
# WHY THE ARMS FACE EACH OTHER rather than standing side by side, which was
# tried first and looks tidier: a UR5e with this tool orientation is not
# symmetric about its own centreline. See APPROACH_WINDOW below. Facing them
# at each other flips arm B's y axis, so the beer between them is at POSITIVE
# y in BOTH frames and both arms work it from a well conditioned pose. Side
# by side, the beer is necessarily on one arm's bad side, and measured, it is
# the side where arm B cannot descend onto the cap.
ARM_B_ORIGIN = (0.61, 0.40, 0.9)
ARM_B_YAW = math.pi

# Arm A's yaw is zero, so arm B's offset in arm A's frame is just the
# difference. Its ORIENTATION in that frame is ARM_B_YAW, which the xacro
# carries separately.
ARM_B_IN_A = (ARM_B_ORIGIN[0] - ARM_A_ORIGIN[0],
              ARM_B_ORIGIN[1] - ARM_A_ORIGIN[1],
              ARM_B_ORIGIN[2] - ARM_A_ORIGIN[2])

COUNTER_Z = 0.9                 # world z of the counter top

# The bar top itself, as the world file places it. Not decoration: the
# planning scene publishes a box of exactly this size and bartender_pour
# restates it in arm A's frame, so a counter that grew here and nowhere else
# is a counter the planner routes a forearm through.
COUNTER_SIZE = (1.76, 1.60)
COUNTER_CENTRE = (0.08, 0.0)

# WHERE AN ARM CAN ACTUALLY TAKE A BOTTLE OFF THE LINE.
#
# This is the constraint that decides the whole layout, it is not obvious,
# and it was found by measurement rather than by reading a datasheet.
#
# bartender_pour grasps from the side with a fixed tool orientation (tool0's
# +z along the arm's +x), and the 2F-85 hanging off the wrist puts the wrist
# centre off the arm's own centreline. So for a flange target at (x, y) in
# the arm's frame, the shoulder pan that reaches it is NOT near the target's
# bearing -- it is rotated off it, always the same way, and by more and more
# as y goes negative. Measured on the running robot, at flange x = 0.305,
# smallest achievable |pan| against the target's bearing:
#
#     flange y   -0.20   -0.15    0.00    0.15    0.25    0.40    0.50   0.60
#     pan        -1.32   -1.32   -0.86    0.09    0.46    0.80    0.93   none
#     bearing    -0.58   -0.46    0.00    0.46    0.69    0.92    1.02   1.10
#
# The two poses this project had already proven -- the old whiskey and cola
# approaches -- sit at pan 0.079 and -0.335, i.e. within 0.23 rad of their
# bearing. Everything from y = 0.10 up to y = 0.50 is in that band. Below
# y = 0, the arm swings a long way past the bearing and folds back on
# itself, and at y = 0.60 there is no branch left at all.
#
# It is not a theoretical concern. A first cut of this bar put the whiskey
# at flange y = -0.20, pan -1.376. The descent and the run-in both planned
# and executed, the gripper stalled at 0.0887 rad -- indistinguishable from
# the 0.0900 the old layout recorded -- and then the bottle slipped straight
# back out during the lift on one run of two.
#
# So every slot a pouring arm has to service is inside this window, and that
# is why the line is five slots rather than nine.
APPROACH_WINDOW = (0.10, 0.50)


def to_arm(xyz_world, origin, yaw):
    """Convert a world point into an arm's base frame."""
    dx = xyz_world[0] - origin[0]
    dy = xyz_world[1] - origin[1]
    dz = xyz_world[2] - origin[2]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy, dz)


def to_world(xyz_arm, origin, yaw):
    """Invert to_arm."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (origin[0] + c * xyz_arm[0] - s * xyz_arm[1],
            origin[1] + s * xyz_arm[0] + c * xyz_arm[1],
            origin[2] + xyz_arm[2])


# -------------------------------------------------------------- the stations

# THE BOTTLE LINE. One world x, five slots along y at a fixed pitch.
#
# Written as a pitch and an occupant list rather than as five coordinates
# because the empty slots are the point: they are real, surveyed, reachable
# positions a bottle can be dropped into, not leftover tabletop. Adding one
# means naming it here and adding its model to the world file at the y this
# computes; nothing else on the bar has to move.
#
# The line is at world x = 0.08, which is 0.53 in front of each arm. That
# depth is set from the other end: the side grasp stands off
# GRIP_AHEAD_OF_TOOL0 + APPROACH_BACKOFF = 0.245 along the arm's +x, so the
# approach flange lands at 0.285 -- near the 0.305 the old layout's whiskey
# approach used, which is the depth this project has actually proven.
#
# HOW LONG THE LINE CAN BE is APPROACH_WINDOW, not the counter. Arm A can
# service world y in [-0.30, +0.10] and arm B, facing the other way, [-0.10,
# +0.30]; the two overlap in the middle, which is how five contiguous slots
# are all servable by at least one arm. Stretching the line further just adds
# positions nothing can reach.
BOTTLE_LINE_X = 0.08
SLOT_PITCH = 0.15
BOTTLE_SLOTS = (
    (-0.30, 'whiskey'),
    (-0.15, 'cola'),
    (0.00, 'beer'),
    (0.15, None),
    (0.30, None),
)

# EACH ARM'S OWN WORKING STATION, off the line and out of everyone's way.
#
# Arm A's is the glass. Arm B's is the opener's holster. They are not a
# shared row and it would be wrong to make them one, because the two arms
# want opposite things from theirs:
#
#   The glass has to be FURTHER out than the line (0.65 in arm A's frame).
#   The pour tilts the bottle about its grip point over the glass, and at
#   POUR_TILT the bottle lies almost flat with its base 0.30 BEHIND the
#   glass at a height of 0.18 -- below the 0.3055 top of anything standing
#   in the line. So the pour sweeps back across the line, and the only thing
#   that makes that safe is putting the glass off every slot's y. It is at
#   world y = -0.55, a quarter of a metre clear of the nearest bottle, and
#   pour_sweep_clearance() is what checks it.
#
#   The holster has to be CLOSER in (0.51 in arm B's frame). Arm B picks the
#   opener by descending vertically onto it from 0.40, and how far out that
#   descent still works is sharply limited: measured, a holster at 0.67 from
#   the base could not be descended to at all -- arm B stopped 346mm short
#   and the goal failed on "arm B could not pick up the opener". 0.51 puts
#   the flange at 0.365, near the 0.407 the old layout used and proved.
STATION_GLASS = (0.20, -0.55)
STATION_OPENER = (0.10, 0.40)

# World (x, y) for everything with a name. The slot table above and the two
# working stations are the only places a position is decided.
# Must match bar_world.sdf, which test_layout.py checks model by model.
STATIONS = dict(
    [(name, (BOTTLE_LINE_X, y)) for y, name in BOTTLE_SLOTS if name],
    glass=STATION_GLASS,
    opener=STATION_OPENER,
)


def slot_y(index):
    """Locate slot `index`, counted from the middle of the line, on world y.

    The middle slot (0) is the beer's: it is the only one equidistant from
    the two arms, which is what the two-armed open needs and what nothing
    else on the bar does.
    """
    return index * SLOT_PITCH


def free_slots():
    """List the world (x, y) of every slot a bottle could still be put in."""
    return [(BOTTLE_LINE_X, y) for y, name in BOTTLE_SLOTS if name is None]


def servicing_arms(xy):
    """Name the arms whose approach window covers a point on the line.

    A slot outside every arm's window is a position on the counter, not a
    station: nothing can take a bottle off it. See APPROACH_WINDOW for what
    the window is and how it was measured.
    """
    lo, hi = APPROACH_WINDOW
    here = []
    for arm, origin, yaw in (('a', ARM_A_ORIGIN, ARM_A_YAW),
                             ('b', ARM_B_ORIGIN, ARM_B_YAW)):
        _x, y, _z = to_arm((xy[0], xy[1], COUNTER_Z), origin, yaw)
        if lo - 1e-9 <= y <= hi + 1e-9:
            here.append(arm)
    return here


def pour_sweep_clearance():
    """Give the smallest gap between the pour's sweep and a standing bottle.

    The poured bottle lies back across the line at the glass's y (see
    STATION_GLASS), at a height below the top of anything standing there, so
    what keeps them apart is this gap and nothing else. What has to fit in
    it is the two bottles' envelopes, which is 0.0946 at worst.
    """
    return min(abs(y - STATION_GLASS[1])
               for y, name in BOTTLE_SLOTS if name)


# ----------------------------------------------------- beer, cap and opener
#
# RESTATED from ros2_ws/src/bartender_gazebo/scripts/make_beer_and_opener.py,
# which is where they are decided and where the reasoning for each of them
# lives. They are restated rather than imported because that script is a
# standalone generator in a different package and is not installed as a
# module; test_layout.py loads it by path and asserts every one of these
# still matches, so the copy cannot drift.
BEER_HEIGHT = 0.2581            # base to the lip
BEER_BODY_RADIUS = 0.0322
BEER_GRIP_BAND = (0.170, 0.200)     # the straight neck the pads close on
BEER_GRIP_RADIUS = 0.0193           # measured off the mesh by the generator
CAP_HEIGHT = 0.0065
CAP_RADIUS = 0.0161
BELL_HEIGHT = 0.021             # rim to the crown plate: full seating depth
BELL_CLEARANCE = 0.0025         # bore over the cap
BELL_CAPTURE = 0.0065           # ...and over the chamfered mouth at the rim
BELL_OUTER_RADIUS = 0.0236
OPENER_HEIGHT = 0.127
OPENER_GRIP_Z = 0.077           # local z of the shaft's middle
OPENER_SHAFT = 0.024            # square, so this is the width either way
HOLSTER_POST_TOP = 0.014

# Height of the opener's origin -- its bell rim -- when it is standing on the
# holster. ZERO: the rim rests flat on the counter and the post stands inside
# the bell as a locator without touching it. Measured in sim before it was
# written down here, and the generator was then changed to make it the only
# possible answer rather than a two-millimetre coincidence; see the
# "WHAT THE OPENER ACTUALLY RESTS ON" note in make_beer_and_opener.py.
OPENER_REST_RIM_Z = 0.0

BEER_STAND_RADIUS = 0.0540      # outer, for the planning scene
STAND_HEIGHT = 0.026

# KEEP-OUT ROUND THE STANDING BOTTLE, for the planner only.
#
# Much bigger than the bottle, and the shape is chosen as carefully as the
# size. A cylinder of the bottle's own 32mm radius does not stop a plan from
# knocking it over, which is not a subtle failure: traced, arm A's open
# fingers crossed the bottle at cap height on the way to a pose directly
# above it, with the fingertip link origins 78mm out from the axis -- outside
# a 32mm envelope -- while the fingertip GEOMETRY reaches another 51mm
# forward, so it swept 17mm inside the glass. A planner cannot be blamed for
# a path through a volume nobody told it about, and the executed trajectory
# cuts corners off the planned one besides.
#
# 100mm of radius covers both. The height is what makes it usable: it stops
# at 0.32, which is above everything the bottle has (0.2646 with its cap) and
# BELOW the 0.40 the arms transit at. So a pose directly over the bottle is
# still legal -- which it must be, since that is where every approach ends --
# while anything that dips towards it on the way is not.
#
# It only exists while the bottle is standing. The moment it is picked up it
# becomes an attached object at its own true size, because then it is
# something to be kept clear of, not something to be kept away from.
BEER_KEEPOUT_RADIUS = 0.10
BEER_KEEPOUT_HEIGHT = 0.32

# --------------------------------------------------------------- rest poses

# Arm A's home, the same configuration as bartender_pour's and the SRDF's
# 'home' state. Not taught here: this server only ever uses it as a place to
# start from and as the IK seed, and the taught version belongs to the pour.
ARM_A_HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

# Arm B's, matching the SRDF's 'b_home'. Now literally a copy of arm A's,
# and that is the point rather than an oversight. The two arms differ by a
# pure translation along arm A's +y, so the configuration that folds one up
# clear of the bar folds the other up clear of it too, in the same direction,
# 0.75 further along. It used to be [0.0, -2.0, 1.6, -1.17, -1.5707963, 0.0]
# because arm B faced back across a narrower counter from a pedestal and arm
# A's angles would have driven it into the worktop.
ARM_B_HOME = list(ARM_A_HOME)

# ------------------------------------------------------------- the grasps

# Distance from tool0 to the point between the pads, along tool0's +z. The
# same number bartender_pour uses (GRIP_AHEAD_OF_TOOL0), and for the same
# reason: it is a property of the 2F-85, not of what is being held.
GRIP_AHEAD_OF_TOOL0 = 0.145

# Half the fingertip pad's span along the bottle's axis, about tool0. A copy
# of render_bartender_urdf.PAD_HALF_WIDTH, kept here for the same reason
# GRIP_AHEAD_OF_TOOL0 is -- this package plans against the gripper without
# depending on the description package, which is CMake and has no importable
# module. The widened pad is symmetric about tool0 by construction, unlike
# the stock one; test_layout.py checks this against BEER_GRASP_HEIGHT.
PAD_HALF_WIDTH = 0.0155

# Where the pads close. NOT the body -- the reasoning is in
# make_beer_and_opener.py under "WHERE THE BOTTLE IS HELD", and the short
# version is that the body is a smooth 64.4mm cylinder and two flat pads
# throw those rather than hold them. This was found the direct way: the first
# run gripped the body and put the bottle 3.8 metres away.
#
# 4mm ABOVE the middle of the neck band, which looks like a fudge and is not.
# The band is 30mm tall and the pad is 31mm tall, so it cannot be centred on
# the band at all: something has to overhang, and the only question is which
# end. The top is free -- above 0.200 the bottle tapers to 34.9mm and then
# 28.9mm, so a pad hanging over it simply never touches. The bottom is not.
#
# Below 0.170 is the shoulder step, 51.0mm against the neck's 38.7mm, and a
# pad reaching onto it grips THAT instead. That is not a theoretical worry.
# Traced by stepping the gripper shut a command at a time: the fingers stop
# dead at a 41.6mm gap and stay there through fifteen further commands
# without the bottle so much as twitching -- a real, stable stall, on the
# shoulder rather than on the neck. It is not even a bad grip. It is just not
# the same grip every time: other runs seated properly on the neck at 34.8mm,
# and a grasp check cannot accept both 41.6 and 34.8 as "the neck" without
# also accepting an empty gripper.
#
# +4mm puts the pad at 0.1735..0.2045: 3.5mm clear of the shoulder and 26.5mm
# of it in contact with the neck.
#
# BE CAREFUL MOVING THIS UP TO BUY PAD WIDTH. It is tempting, because the
# pad's width is capped by exactly this clearance (see "HOW WIDE, AND WHAT
# SETS THE LIMIT" in render_bartender_urdf.py) and 2mm here buys 2mm there.
# It was tried, at 0.191 with a 33mm pad, and the two clean runs it got both
# failed, with the grasp measurably DEEPER -- the fingers closing 6.5 and
# 6.6mm into the 38.7mm neck, against 1.0-5.3mm over eight runs at 0.189.
# Two runs is not a result and it is not recorded here as one; what it is, is
# a reason not to assume the 30mm band is interchangeable with itself 2mm up,
# and a reason to re-measure the open end to end if this number is changed.
#
# (A third run at 0.191 looked like more of the same and is NOT counted: an
# orphaned `ign gazebo server` from the previous cycle was still on the
# transport bus. The harness now refuses to start if ANY simulator survives,
# not just one whose command line names the world -- the orphan's does not.)
BEER_GRASP_HEIGHT = 0.189

# Nothing here says what to COMMAND the gripper to, and that is deliberate.
# Arm.grasp closes until the fingers stop and takes the angle they stopped at
# as the grasp, so the only thing it needs from this file is how wide the
# thing is. See the docstring there for why a measured stall beats a
# predicted one; the short version is that a predicted one is indisputable
# only when it is right.
#
# For reference, since it is the number that used to live here: the 38.7mm
# neck touches at knuckle 0.460 by the pad-gap curve in arm.py, and the joint
# was measured stalling at 0.4995 and holding there through a lift and a
# return.
BEER_WIDTH = 2.0 * BEER_GRIP_RADIUS

# One more thing the curve explains, because it looks like a bug when you
# first see it: while the gripper is still CLOSING, the reported joint angle
# runs about 30% short of the command (commanded 0.68, reading 0.4866 a tenth
# of a second later) and only converges once the command stops moving. Any
# grip check has to be made after a settle, never between steps.

# How far the beer is lifted out of its well to be opened, measured at the
# base. The well is 26mm deep, so this clears it by 24mm: enough that the bottle is unambiguously
# held by the arm and not resting, and low enough that arm B is not reaching
# up at full stretch to get over the cap. Higher looks more impressive and
# costs reach on both arms at once.
HOLD_LIFT = 0.050

# BOTH GRASPS HERE DESCEND STRAIGHT DOWN onto the object, rather than
# dropping beside it and running in horizontally the way bartender_pour does.
#
# That run-in is not decoration in the pour: the whiskey bottle is 77.2mm
# across in an 85.00mm opening, 3.9mm a side, which is less than the
# Cartesian error a joint-space goal can leave behind, so the pads have to be
# placed exactly and then translated in. Neither object here is anything like
# that tight -- the beer is 64.4mm (10.3mm a side) and the opener's shaft is
# 24mm (30mm a side) -- so a vertical descent with the object already between
# the open pads has margin to spare.
#
# And it avoids a real failure. A horizontal run-in needs a stand-off pose
# 100mm back along the arm's own +x, and the beer sits 586mm from arm A: back
# the 145mm-long gripper off from there and tool0 is folded in towards the
# robot's own shoulder. Measured on the old layout, where the beer was
# 434mm out and the stand-off landed 269mm from the base, that pose plans
# but cannot be descended to -- "beer descend only reached 0.00 of the
# path". Coming down from above never goes near it.
# Transit height for both arms, above the counter.
#
# 0.40, and the 100mm over what looks necessary is the point. A capped beer
# standing in its well reaches 0.2646, so 0.30 clears it by 35mm at tool0 --
# except that the gripper hangs about 30mm below tool0, which leaves 5mm, and
# 5mm is inside what a joint-space move overshoots by. Measured: the approach
# plan validated clean and the bottle was knocked flat at the end of the
# move, intermittently, which is the signature of clipping something on
# overshoot rather than planning through it.
APPROACH_Z = 0.40

# ------------------------------------------------- crossing the counter
#
# APPROACH_Z above is for an EMPTY gripper. Carry something and the height
# that matters is not the flange's, it is the lowest point of what is in the
# fingers -- and the opener hangs a long way down.
#
# WHAT WENT WRONG, on the layout this replaced. The opener is gripped
# OPENER_GRIP_Z (77mm) above its bell rim, so at a flange height of 0.40 the
# rim is at 0.323. The tallest thing standing on the counter is the cola's
# pour spout at 0.3055 and the whiskey's at 0.3005, which leaves 17.5mm and
# 22.5mm. That is thin on its own; what made it a collision rather than a
# near miss is that NOTHING STOPS IT. bartender_open's planning scene
# contains the beer, its stand and the opener station -- it has never
# contained the whiskey or the cola, so a path straight through either of
# them validates clean.
#
# And the first cross-counter move starts far lower than 0.40: the pick
# lifts the opener PICK_LIFT (50mm) off its post, putting the rim at 0.050,
# and the next move is a joint-space swing to over the cap. On the old
# layout the holster sat 112mm from the whiskey, so that swing started with
# the bell below the whiskey's shoulder and a hand's breadth away from it.
#
# MEASURED there, same goal run twice from a fresh simulator, watching the
# bottles' own poses on the pose stream:
#
#     flange height    whiskey moved    cola moved
#     0.40 (old)           71.7mm         11.8mm
#     OPENER_TRANSIT_Z      0.0mm          0.0mm
#
# 71.7mm is not a nudge. The whiskey ended 72mm ABOVE the counter -- the
# opener hooked it on the way past and carried it, and it was still climbing
# when the run ended. The cola, 200mm further away, was caught too.
#
# THE REDESIGNED BAR TAKES MOST OF THAT AWAY, and the height stays anyway.
# The holster is now 0.90m from the whiskey and 0.66m from the cola, on the
# serving strip rather than in among the bottles, and arm B's half of the
# line is empty -- so the geometry that produced those two numbers is gone.
# The opener's own crossing, holster to over the beer, clears the nearest
# bottle by 250mm.
# What has NOT changed is the reason it was able to happen: the whiskey and
# the cola are still not in this action's planning scene, so nothing refuses
# a path through them. OPENER_TRANSIT_Z is the guard that does not depend on
# where the bottles are, which is exactly why it is worth keeping after
# moving them. See the note in open_action_server._publish_obstacles.

# Tallest thing standing on the counter, from the counter top. The cola's
# pour spout. Whiskey 0.3005, capped beer 0.2646 -- both shorter, and this
# has to clear whichever is worst.
COUNTER_TALLEST = 0.3055

# Air left under a carried load while crossing. The same 100mm APPROACH_Z
# allows itself, and for the same reason: a joint-space move overshoots at
# the end of a transit, and anything inside that is knocked over
# intermittently rather than never.
TRANSIT_CLEARANCE = 0.10

# Flange height for crossing the counter with the opener in the fingers.
# Derived, so that regripping the opener somewhere else along its shaft --
# changing OPENER_GRIP_Z -- moves this with it instead of silently eating
# the clearance.
OPENER_TRANSIT_Z = COUNTER_TALLEST + OPENER_GRIP_Z + TRANSIT_CLEARANCE

# ------------------------------------------------------------ the press
#
# How far the opener is driven past the point where its crown plate is
# already flat on the cap. There is nowhere for it to go, so this is not a
# distance the arm travels, it is a position error the trajectory controller
# holds against the bottle -- which is exactly what "push" means for a
# position-controlled arm. 6mm is enough to be unambiguous and small enough
# that arm A's grip carries it; the whole load ends up on the pad friction
# that also carries the bottle's weight during a pour.
PRESS_TRAVEL = 0.006

# The seating test. The bell is 21mm deep -- a 5mm chamfered mouth and then
# 16mm of straight bore -- so a fully seated opener has its rim 21mm below
# the top of the cap. Requiring 14mm puts the cap past the chamfer and into
# the bore, which is the thing that actually means "over the cap" rather than
# "resting on it", while leaving room for the arm to stop a little short
# under load.
SEAT_DEPTH_MIN = 0.014
# ...and laterally, the mouth funnels in anything inside BELL_CAPTURE, so
# beyond that it is not going over the cap at all.
SEAT_OFFSET_MAX = BELL_CAPTURE

# THE GATE: what has to be true before the cap is allowed to come off.
#
# Two measurements, and they answer different questions. Neither is enough on
# its own, and the first version of this got that wrong in a way worth
# recording, because it looked fine.
#
# MIN_HELD_CLEARANCE asks: is arm A holding the bottle AT ALL? It is the
# bottle's base above the counter, and it is the only thing here that a
# bottle standing in its well cannot fake. The original gate was the shift
# measurement alone, and a standing bottle passes that easily -- push down on
# something already resting on a counter and it does not move, because the
# counter takes the load. The whole claim of this sequence is that one arm is
# holding what the other pushes on, so the check for it should be the
# obvious one: the bottle is in the air.
#
# It is read straight after the LIFT, not after the press. Read after the
# press it condemns runs that are working: two position-controlled arms with
# 6mm of interference between them deflect, and the one being pushed on goes
# down by 20mm or so. That is the push doing its job.
#
# BOTTLE_SHIFT_MAX asks: did the push go into the cap or into moving things?
# 25mm, which is generous on purpose. The bottle hangs from friction on a
# round neck and the arm holding it is a position-controlled machine with
# compliance of its own, so some bodily movement under a 6mm interference is
# not the grip failing -- it is what pushing on something looks like.
# Measured across successful opens: 3.7mm, 4.3mm, 10.1mm, and one at 19.5mm
# that seated the bell 20.8mm of a possible 21. What this has to catch is the
# bottle being knocked off the stand or out of the fingers, which is tens of
# millimetres and usually ends on the floor.
#
# There is a third check that is not a number: arm A's fingers must still be
# stalled where they stalled when they took hold. See Arm.still_holding.
MIN_HELD_CLEARANCE = 0.030
BOTTLE_SHIFT_MAX = 0.025

# HOW THE FREED CAP GETS OFF THE BOTTLE.
#
# It does not, on its own. The cap is a disc resting on a 27.3mm lip (see
# make_beer_and_opener.py for why it is a disc and not a skirt), the bottle
# is being held upright, and a disc on a flat lip is a stable perch. Detached
# and left alone it settled 13mm from the mouth, leaning on the neck.
#
# Sweeping it off sideways with the opener's own bell was tried first and is
# not reliable: the bell has 2.5mm of radial clearance on the cap, so how
# much of the cap it still has hold of depends on exactly how far it has been
# lifted off it, and runs came back at 13, 21, 28, 55 and 113mm.
#
# So arm A tips the bottle instead. The arm holding the bottle is the obvious
# thing to ask, and it is what a person checking would do.
#
# HOW FAR is not a matter of taste. A disc on a tilted lip slides when the
# tilt passes the friction angle, atan(mu), and the cap's mu against glass is
# 0.9 -- so it needs more than 42 degrees, and 40 is not enough. That is not
# a calculation done afterwards to explain a failure: a run at 40 degrees
# left the cap sitting there, 12mm from the mouth, having been detached
# perfectly well. 60 degrees gives tan 1.73 against a mu of 0.9, which is
# most of a factor of two in hand.
#
# The bottle stays well clear at that angle -- its base ends up 145mm above
# the counter and its mouth 33mm clear of the whiskey stand -- and the pour
# takes these same bottles to 97 degrees, so the gripper is not being asked
# anything new.
#
# The tilt is interpolated rather than commanded in one move, for the same
# reason the pour's is: a single large rotation of a held bottle is a swing,
# not a turn.
SHED_TILT = 1.05                # radians, 60 degrees
SHED_STEPS = 8
SHED_SETTLE_S = 1.5

# After that, how far the cap must have got from the bottle's mouth to count
# as off. A cap still sitting on the lip reads ~0.
# 30mm is past any settling and well short of the ~200mm it travels when it
# drops to the counter.
CAP_FREE_MIN = 0.030


def cap_bottom_above_base():
    """Local z of the cap's underside on a standing bottle."""
    return BEER_HEIGHT


def cap_top_above_base():
    return BEER_HEIGHT + CAP_HEIGHT


def station_in_arm(name, arm):
    """Locate a station, as (x, y, counter-level z), in arm 'a' or 'b'."""
    x, y = STATIONS[name]
    origin, yaw = ((ARM_A_ORIGIN, ARM_A_YAW) if arm == 'a'
                   else (ARM_B_ORIGIN, ARM_B_YAW))
    return to_arm((x, y, COUNTER_Z), origin, yaw)


def tilted_tool0(grip_xyz, theta):
    """Flange pose that tilts a grasped object about its own grip point.

    The grip point stays where it is and the object turns around it, which is
    the same trick bartender_pour's pour tilt uses -- it just needs a lot
    less of it here.

    theta is a rotation about the arm's +y, which side_quat expresses as an
    orientation. At theta=0 tool0's +z lies along the arm's +x, so the grip
    point is GRIP_AHEAD_OF_TOOL0 along +x from the flange; rotating sends
    that offset to (cos, 0, -sin), and the flange goes wherever that puts it.
    Positive theta tips the object's top away from the arm's base, which is
    the direction with nothing under it.
    """
    return (grip_xyz[0] - GRIP_AHEAD_OF_TOOL0 * math.cos(theta),
            grip_xyz[1],
            grip_xyz[2] + GRIP_AHEAD_OF_TOOL0 * math.sin(theta))


def side_grasp_tool0(grip_xyz):
    """tool0 position for a side grasp whose grip point is at grip_xyz.

    In the side grasp tool0's +z is the approach direction and points along
    the arm's own +x, so tool0 sits GRIP_AHEAD_OF_TOOL0 back along x from the
    point between the pads. Orientation is SIDE_QUAT, which the node applies;
    this is only the translation, and it is the whole of the difference
    between "where the pads must be" and "where to send the flange".
    """
    return (grip_xyz[0] - GRIP_AHEAD_OF_TOOL0, grip_xyz[1], grip_xyz[2])


def beer_grip_point(lift=0.0):
    """Where arm A's pads close on the beer, in arm A's frame."""
    x, y, _ = station_in_arm('beer', 'a')
    return (x, y, BEER_GRASP_HEIGHT + lift)


def opener_grip_point(rim_z):
    """Where arm B's pads close on the opener shaft, in arm B's frame.

    rim_z is the height of the opener's own origin -- the bell rim -- above
    the counter, because that is what moves: on the holster it is
    OPENER_REST_RIM_Z, over the cap it is wherever the bell has got to.
    """
    x, y, _ = station_in_arm('opener', 'b')
    return (x, y, rim_z + OPENER_GRIP_Z)


def opener_over_cap_grip(rim_z):
    """Predict the grip point above the beer station, in arm B's frame.

    The design value, not what the sequence uses: open_action_server aims
    the bell at the cap's measured pose instead, because the bottle rolls a
    few millimetres under the closing pads and this would be that far off.
    Kept because it is what the layout PREDICTS, which is what the tests
    check the layout against.
    """
    x, y, _ = station_in_arm('beer', 'b')
    return (x, y, rim_z + OPENER_GRIP_Z)


def seated_rim_z(lift=HOLD_LIFT):
    """Nominal rim height, above the counter, with the plate flat on the cap.

    The bottle's base is `lift` above the counter, its cap top is
    cap_top_above_base() above that, and a seated bell has its rim one bell
    depth below the cap top.
    """
    return lift + cap_top_above_base() - BELL_HEIGHT


def reach(xyz):
    """Straight-line distance from an arm's base. Sanity only."""
    return math.sqrt(sum(v * v for v in xyz))
