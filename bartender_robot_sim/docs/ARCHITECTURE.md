# Architecture

How the bartender is put together, and where to add things.

Read this before adding a package. The two invariants in "What holds it
together" are the reason the project has stayed workable; most of the bugs
worth writing down came from breaking one of them.

## The stack

```
        operator            VLM / planner / other agent
            |                        |
            |  browser               |  JSON over HTTP        <- docs/CONTROL_API.md
            |                        |     (Phase B built; Phase C started --
            |                        |      simple movement only)
   +--------v----------+    +--------v---------+
   |  teach pendant    |    |  bartender_api   |
   |  (GUI + terminal) |    |                  |
   +--------+----------+    +--------+---------+
            |                        |
            +-----------+------------+
                        |  ROS 2 actions / services / topics
         +--------------v---------------+
         |  SKILLS                      |
         |  bartender_pour  PourDrink   |   arm A: pick, tilt, pour, return
         |  bartender_open  OpenBottle  |   both arms: hold and press
         +--------------+---------------+
                        |
         +--------------v---------------+
         |  MOTION                      |
         |  MoveIt 2 (move_group)       |   planning, IK, planning scene
         |  ros2_control                |   trajectory + gripper controllers
         +--------------+---------------+
                        |
         +--------------v---------------+
         |  ROBOT + WORLD               |
         |  bartender_description  URDF |   2x UR5e + Robotiq 2F-85
         |  bartender_gazebo       SDF  |   the bar, bottles, glass, opener
         +------------------------------+

   bartender_open/layout.py  ---- feeds every layer above ---->
        geometry, stations, reachability. No ROS in it.
```

## What holds it together

Two rules. Everything else is detail.

### 1. One source of geometric truth

`bartender_open/layout.py` decides where everything is and which arm can
reach it. It imports no ROS, so it can be checked without a simulator, a
build or a running `move_group` — and `bartender_open/test/test_layout.py`
checks it against every *copy* of those numbers: the world SDF, the URDF's
arm-B mount, the launch file's spawn pose, and `bartender_pour`'s own
restatement of its three stations.

Copies exist because ROS packages cannot always import each other
(`bartender_pour` cannot import `bartender_open`). The answer is not to
delete the copies but to make them fail loudly when they drift — the tests
parse the other files as text and compare.

**If you move something, move it in `layout.py` and let the tests tell you
what else has to follow.**

### 2. Skills report measured evidence, not booleans

`OpenBottle` does not return "it worked". It returns how far the cap ended
up from the mouth and how far the bottle moved while it was being pushed
on. A sequence that ran to the end with the cap still on the lip reads near
zero and **fails**.

This is what makes the system debuggable, and it is what will make it
usable by an autonomous caller. Keep it: a new skill should report the
quantity that proves it did its job, not the fact that its state machine
reached the last state.

## The frame problem

The single largest source of bugs here. There are three frames:

| frame | what it is |
|---|---|
| `world` | Gazebo's. The SDF places everything in it. Not a TF frame anything plans against. |
| `base_link` | **arm A's** base, at world `(-0.45, -0.40, 0.9)`. `bartender_pour` plans everything in it. |
| `b_base_link` | **arm B's** base, at world `(0.61, 0.40, 0.9)`, yawed 180°. |

The arms face each other, so **their x and y axes point opposite ways**. A
pose read or jogged in the wrong frame lands most of a metre away, in the
wrong direction, and nothing errors. `layout.to_arm()` / `to_world()` are
the only sanctioned conversions.

Both arm bases sit at `z = 0.9`, which is the counter top, so a `z` in
either arm's frame is "height above the counter" and the two agree. That is
a property the layout was chosen to keep, not a given.

## Reachability is not symmetric, and it decides the layout

A UR5e carrying this gripper, with the fixed side-grasp orientation, is not
symmetric about its own centreline — the wrist hangs to one side. Measured
on the running robot, the shoulder pan needed for a flange target is rotated
off the target's bearing, increasingly so as y goes negative:

```
flange y   -0.20   -0.15    0.00    0.15    0.25    0.40    0.50   0.60
pan        -1.32   -1.32   -0.86    0.09    0.46    0.80    0.93   none
bearing    -0.58   -0.46    0.00    0.46    0.69    0.92    1.02   1.10
```

The usable band is `APPROACH_WINDOW = (0.10, 0.50)` in the arm's own frame.
In world terms:

- **arm A** serves `y ∈ [-0.30, +0.10]`
- **arm B** serves `y ∈ [-0.10, +0.30]`

That is why the arms face each other rather than standing side by side, and
why the bottle line is five slots and not nine. Any API that offers "reach
for X" has to answer against this, not against nominal arm reach.

**Consequence worth knowing before you plan work:** arm A is the only arm
that can pour, because the glass sits 0.667 m from arm A's base and
**1.035 m** from arm B's — past the UR5e's 0.85 m reach. The two free slots
in the line are on arm B's side, so a bottle placed there today can be
picked up and not poured. See `docs/ROADMAP.md`.

## Packages

| package | what it owns |
|---|---|
| `bartender_description` | xacro: 2× UR5e + 2F-85, ros2_control wiring. The fingertip pads are *generated* by `scripts/render_bartender_urdf.py`, not written by hand. |
| `bartender_gazebo` | the bar world (SDF), model generators, sim launch and spawn |
| `bartender_moveit_config` | hand-written MoveIt config (see its README for why) |
| `bartender_bringup` | one launch tying sim + control + MoveIt + both skills |
| `bartender_pour_interfaces` | the `PourDrink` and `OpenBottle` action definitions |
| `bartender_pour` | the pour skill. **Arm A only**, plans in `base_link` |
| `bartender_open` | the beer-opening skill, both arms — and `layout.py` |
| `bartender_teach` | teach pendant (terminal + browser), tool frames, taught points and pipelines |
| `bartender_api` | HTTP/JSON control API: `GET /world`, `GET /state`, `POST /can`, error taxonomy (read-only), plus `POST /move/point`, `POST /move/jog`, `POST /gripper` (simple movement, over `Pendant.dispatch()`). See `docs/CONTROL_API.md` |
| `models/` | Gazebo models; several are **generated** — edit the script, re-run it, don't hand-edit the SDF |
| `fingertip/` | a parametric CadQuery generator for a neck-hooking fingertip (standalone, own venv) |

## Where to add a new ...

**A bottle.** `layout.BOTTLE_SLOTS` (name a free slot), a model under
`models/`, an include in `worlds/bar_world.sdf` at the y the slot computes.
`test_layout.py` will tell you what else to touch. Check
`layout.servicing_arms()` says an arm that can *use* it reaches it.

**A skill.** New package + a new action in `bartender_pour_interfaces`.
Import `layout` for geometry; publish your obstacles into the planning scene
yourself — do **not** rely on another server having done it (that exact
assumption put arm B's forearm 236 mm below the worktop). Return a measured
quantity in the result.

**A front end.** Do not talk to MoveIt. Go through a skill action, or
through `Pendant.dispatch()` for jogging. The browser pendant owns no robot
logic at all and that is why a bound or a refusal fixed once reaches both
front ends.

**An arm.** `teach_points.ARMS` is a table built from a prefix; the
description uses `b_` for arm B. Nothing in the pendant hard-codes two.

## Simulation vs. real hardware

The boundary is `ros2_control`. Everything above it — MoveIt, the skills,
the pendant, the layout — is hardware-agnostic. Swapping in a real UR5e
means a different hardware interface and a recalibrated `layout.py`; it does
not mean touching the skills.

What is sim-specific and will need rethinking on hardware:

- `DetachableJoint` for the beer cap (Gazebo-only, and it cannot re-attach —
  one open per simulator run).
- Model poses arrive on `/world/bar_world/dynamic_pose/info`, which is
  ground truth. On hardware this becomes perception, and it becomes
  uncertain. The API in `docs/CONTROL_API.md` is shaped to survive that
  change: every pose it reports carries a source and a confidence.

## Known defects that shape the design

Do not design around these being fixed. They are in `docs/ROADMAP.md` with
detail, but in short:

- **Moves that end in contact cannot arrive, and say so.** The arm
  controllers now carry per-joint goal tolerances, so a move that does not
  get there fails (this was a real defect: they used to report success
  wherever they stopped). The four moves that legitimately end in contact —
  setting a bottle down, the opener onto its post, the beer into its stand,
  the bell onto the cap — pass `may_stall`. If you add a move that ends by
  touching something, it needs the same, and its correctness has to be
  proved by a measurement rather than by the controller's opinion.
- **A joint parked on its limit stops responding.** The gripper knuckle,
  left at rest on its lower stop (0.0), never moves again for the rest of
  the simulator run — goals still accepted, controller still active. This
  was the "grippers stop following" defect and it is fixed by never asking
  for 0.0: `GRIPPER_OPEN_POS` is 0.02 in all three packages that command a
  gripper, and anything outside `0.02..0.8` is refused. If you add an
  actuator, keep its resting positions off both stops.
- **The cola slips out mid-pour** intermittently, at the 97° pour angle.

Any autonomous caller must therefore treat "the action returned success" as
weaker evidence than "the measurement in the result is in range" — which is
exactly why rule 2 above exists.
