# Bartender Robot

A two-armed bartender -- two UR5e arms, each with a Robotiq 2F-85 -- taught
to pour a drink and to open a beer in Gazebo Sim simulation, before anything
touches a real robot or a user-facing interface. This file covers setup and
day-to-day commands.

**New here, or building something on top of this?** Start with
[docs/](docs/README.md):
[ARCHITECTURE](docs/ARCHITECTURE.md) (how it fits together and where to add
things), [CONTROL_API](docs/CONTROL_API.md) (the proposed HTTP/JSON API for
a VLM or other non-ROS caller), [ROADMAP](docs/ROADMAP.md) (what works, what
is blocking autonomy, what is next) and
[CONTRIBUTING](docs/CONTRIBUTING.md) (build, test, and the house rules).
[PLAN.md](PLAN.md) is the superseded Phase 1 plan, kept as history.

The two arms stand on one counter facing each other down it, with the bottle
line between them; see "The bar" below for the layout and for the three
measurements that fix it.

They are **not** named symmetrically. Arm A keeps ur_description's
bare joint and link names (`shoulder_pan_joint`, `tool0`, `base_link`) and
arm B has everything under a `b_` prefix. That is deliberate and the
reasoning is at the top of
`bartender_description/urdf/bartender.urdf.xacro`; the short version is that
renaming arm A would rename the planning frame, both controller configs, the
SRDF, the pour server's constants and the user's own taught points, to buy
symmetry in a file nobody reads twice.

**Status: both skills work end-to-end in simulation.** `pour_drink` runs
its pick -> tilt -> pour -> return cycle on arm A; `open_bottle` runs the
two-armed sequence and takes the cap off the beer. See "Status" below for
what is measured and where it still fails.

## One-time setup

```bash
sudo apt-get update && sudo apt-get install -y \
  ros-humble-moveit \
  ros-humble-ur-description \
  ros-humble-ur-simulation-gz \
  ros-humble-ur-moveit-config \
  ros-humble-ur-robot-driver \
  ros-humble-robotiq-description \
  ros-humble-robotiq-controllers \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`scripts/verify_description.sh` checks the xacro macro names/params this
project assumes against what's actually installed -- useful if you update
any of the above packages later and something stops rendering.

## Package layout

- `bartender_description` -- xacro composing UR5e + Robotiq 2F-85 +
  ros2_control/gz_ros2_control wiring.
- `bartender_gazebo` -- the bar world (SDF) and sim launch/spawn.
- `bartender_moveit_config` -- hand-written MoveIt config (see its own
  README.md for why, and for a known RViz-only display bug).
- `bartender_bringup` -- top-level launch tying sim + control + MoveIt +
  the pour skill together.
- `bartender_pour_interfaces` -- the `PourDrink` and `OpenBottle` action
  definitions.
- `bartender_pour` -- the pour skill's action server (pick -> tilt -> pour
  -> return state machine). Arm A only.
- `bartender_open` -- the beer-opening skill: both arms, one holding the
  bottle while the other presses the opener onto its cap. `layout.py` holds
  every pose and dimension it depends on and has no ROS in it, so it can be
  checked without a simulator.
- `bartender_teach` -- teach pendant, GUI and tool frames.
- `models/` -- Gazebo models. Hand-written: `bar_counter`,
  `jack_daniels_bottle`, `cola_bottle`, `serving_glass`.
  Generated, by the scripts in `bartender_gazebo/scripts/` -- edit those and
  re-run them, not the SDF: `whiskey_stand`, `cola_stand`, `beer_stand`,
  `beer_bottle`, `beer_cap`, `bottle_opener`, `opener_holster`.

## The bar

One counter, 1.76 x 1.6, top at world z = 0.9, with **both arms bolted to
it**. Arm A stands at one end facing down the bar and arm B at the other
facing back at it; between them runs a five-slot **bottle line** at a 0.15
pitch, and each arm has its own working station off the line -- the glass
for arm A, the opener's holster for arm B.

```
   y
   ^
+0.8|  +-----------------------------------------------+  counter edge
    |  |                                               |
+0.4|  |                      []  holster        (B)   |  arm B, faces -x
    |  |                      o   free                 |
    |  |                      o   free                 |
 0.0|  |                      #   beer   <- both arms  |
    |  |                      #   cola                 |
    |  |                      #   whiskey              |
-0.4|  |  (A)                                          |  arm A, faces +x
    |  |                        U   glass              |
-0.8|  +-----------------------------------------------+
    +--------------------------------------------------------> x
      -0.45                 0.08 0.20            0.61
                            line
```

Three things about it are not free choices, and each is written up where it
is decided:

- **Why the line runs across the arms rather than along them.** The pour
  grasps from the side, running the gripper in along the arm's own +x, so
  slots at a common x each get a clear lane and no bottle stands behind
  another.
- **Why it is five slots and not nine** -- `APPROACH_WINDOW` in
  `bartender_open/layout.py`. A UR5e carrying this gripper is not symmetric
  about its own centreline, so the band of the line an arm can actually take
  a bottle off is about 0.40 wide. Past it the shoulder swings a long way
  past the target's bearing, and measured, a bottle grasped from such a pose
  slips back out during the lift. The two arms face each other so their
  bands overlap in the middle, which is what makes all five slots servable
  and the shared beer well conditioned for both.
- **Why the glass is off every slot's y.** The pour lays the bottle back
  0.30 behind the glass at 0.18 above the counter, below the top of anything
  standing in the line, so the pour sweeps across the line. The glass sits
  0.25 clear of the nearest bottle, and the carry to it goes over the line
  rather than through it (`CARRY_MOUTH_Z`, raised from 0.40 to 0.61 for
  exactly this).

## Running the sim

```bash
# scene + robot spawn only (no controllers/MoveIt/pour skill):
ros2 launch bartender_gazebo sim.launch.py

# full stack: sim + controllers + move_group + pour_drink action server
ros2 launch bartender_bringup bartender_sim.launch.py
```

Once the full stack is up (give it ~40s -- Gazebo, four controller spawners,
and move_group all start with deliberate delays so they don't race each
other), trigger a pour:

```bash
ros2 action send_goal /pour_drink bartender_pour_interfaces/action/PourDrink \
  "{bottle_id: jack_daniels_bottle, glass_id: serving_glass, pour_amount_ml: 45.0}" \
  --feedback
```

...or open the beer, which uses both arms:

```bash
ros2 action send_goal /open_bottle bartender_pour_interfaces/action/OpenBottle \
  "{bottle_id: beer, stow_after: true}" --feedback
```

### How the beer gets opened

Arm B takes the opener off its post. Arm A lifts the beer out of its stand
by the NECK -- not the body, which is a smooth 64.4mm cylinder that two flat
pads throw rather than hold -- and holds it 50mm clear. Arm B brings the
opener's bell down over the cap and then keeps going 6mm, which is a
position error the controller holds against a bottle the other arm is
holding still. That is the "one arm pushes on the other" part, and it is
where the load goes.

The cap then comes off **because this node publishes on a topic**: it is a
separate model held on by an Ignition `DetachableJoint`, because prying a
crown cap is not something the physics at this fidelity is going to do.
What is *not* scripted is whether that happens. Before publishing, the node
reads the simulator's own pose stream and requires that the bell is really
seated over the cap, that the bottle has not moved much, that arm A's
fingers are still stalled where they closed, and -- the check a bottle
standing in its well cannot fake -- that the bottle is off the counter.
Afterwards it measures where the cap actually ended up, and reports success
on that rather than on having run to the end.

`DetachableJoint` cannot re-attach in Fortress, so a second goal on the same
beer is refused with a message saying to restart the sim.

If a `ros2` CLI command run immediately after a fresh `colcon build`
reports "The passed action type is invalid" or otherwise misbehaves, it's a
transient ament-index/discovery glitch, not a real error -- just retry it.

## The workcell (one arm, the real robot)

The robot that actually exists is **one UR5e with the 2F-85 on a 1.40 x 0.70
table**, its base 0.35 in from one end and 0.35 in from the long side, so it
sits on the table's centreline and has 1.05 of table in front of it:

```
   y
   ^
0.70 +------------------------------------------+
     |                                          |
0.35 |   (UR5e) -> +x                           |
     |                                          |
   0 +------------------------------------------+--> x
     0   0.35                                 1.40      (table corner = `world`)
```

It is set up **alongside** the two-armed bar, not instead of it: the bar and
its skills are untouched and still launch as above. The workcell has its own
description (`bartender_description/urdf/workcell.urdf.xacro`), SRDF,
controller configs, Gazebo world and two bringup launches. The one arm is
arm A with its bare names (`ur_arm_controller`, `ur_manipulator`,
`base_link`), so MoveIt, `bartender_teach` and anything taught carry over.

The **table is a link of the robot description**, not only a Gazebo model,
so MoveIt refuses to plan into it on the real robot too, from the first
moment and without anything having to publish it. Checked: a goal that puts
the wrist in the table fails with "contact between 'workcell_table' and
'wrist_1_link'".

```bash
# simulation
ros2 launch bartender_bringup workcell_sim.launch.py

# the real stack with NO robot: mock hardware, same controllers and MoveIt
ros2 launch bartender_bringup workcell_real.launch.py use_fake_hardware:=true

# the real robot (10.42.0.100 by default)
ros2 launch bartender_bringup workcell_real.launch.py

# the real robot, with a Gazebo twin that copies it live
ros2 launch bartender_bringup workcell_twin.launch.py
```

In the twin, the real robot leads and Gazebo follows: `twin_mirror` streams
the real joint states onto the simulated arm (under `/twin`), and nothing in
Gazebo can move the robot. Checked with the mock robot: after teach-pendant
jogs the twin's six joints matched to 0.0000 rad, and its gripper followed
0.4 -> 0.02 -> 0.7.

Then drive it with the teach pendant tool, `ros2 run bartender_teach teach`
(or `teach_gui`); see `bartender_teach/README.md`. On the real robot the
pendant also powers it on (`robot on`), plays the program (`robot play`),
sets the speed slider (`speed 20`) and switches freedrive (`freedrive on`).

Two numbers are **guesses until someone measures the real cell**, and both
are launch arguments: `arm_yaw` (which way the base faces; the xacro header
says how to check it, and that the UR teach pendant's Base frame is turned
180 degrees from ROS's `base_link`) and `table_height` (0.75). The gripper is
**mocked** on the real robot until it is decided whether it is wired through
the UR's tool connector or its own USB adapter (`gripper_fake_hardware:=false`
plus `use_tool_communication:=true` for the former).

### Connecting the real robot over Ethernet

Step by step, with moving the arm and recovery, in
[docs/WORKCELL.md](docs/WORKCELL.md). In short:

1. **Install the driver**: `sudo apt install ros-humble-ur-robot-driver`, or
   rebuild the Docker image, which now includes it. The container already
   uses host networking, so nothing else changes there.
2. **Put the PC and the robot on one subnet.** The robot is `10.42.0.100`
   (set on the UR pendant: Settings > System > Network, static, netmask
   255.255.255.0). This PC's wired port `eno1` is `10.42.0.67`, no gateway:
   ```bash
   sudo nmcli con add type ethernet ifname eno1 con-name ur5e \
     ipv4.method manual ipv4.addresses 10.42.0.67/24 ipv6.method disabled
   sudo nmcli con up ur5e
   ping 10.42.0.100
   ```
3. **Install the External Control URCap** on the pendant. It ships with the
   driver: `$(ros2 pkg prefix ur_robot_driver)/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap`,
   copied to a USB stick. In Installation > URCaps > External Control, set
   **Host IP to this PC's address** (10.42.0.67) and port 50002.
   Make a program whose only node is External Control and save it.
4. **Let the robot call back.** The driver listens on TCP 50001-50004; if a
   firewall is on (`sudo ufw status`), allow those from the robot's IP.
5. **Launch.** Our robot is in Remote Control mode and the launch defaults
   to `headless_mode:=true`, so no pendant program is needed. The driver
   prints *"Robot connected to reverse interface. Ready to receive control
   commands."* when it is in control. (In Local mode: `headless_mode:=false`
   and press play on the External Control program on the pendant.)
6. **First moves**: speed slider at 10-20%, a hand on the e-stop, and check
   `arm_yaw` before anything else. MoveIt's acceleration limits in
   `joint_limits.yaml` were tuned for sim; the slider scales them down,
   because `ur_arm_controller` is the driver's speed-scaled trajectory
   controller.

Worth doing once it is connected, not before: extract the robot's factory
kinematic calibration with the `ur_calibration` package and pass it in, so
MoveIt's arm matches this particular arm to the millimetre and not only the
nominal UR5e.

## Status / what's still placeholder

- **Verified working end-to-end**: both arms spawn in the bar world; all
  four controllers come up active; `move_group` plans and executes for
  either arm; the `pour_drink` action runs its whole pick -> tilt -> pour ->
  return cycle; the `open_bottle` action runs the two-armed sequence above
  and takes the cap off.
- **On the redesigned bar** (the counter, the bottle line and both arms'
  positions all moved), re-measured from fresh simulators: `pour_drink`
  2/2 full whiskey-and-cokes and `open_bottle` 2/2 caps off. The grips came
  back at the values the old layout recorded -- whiskey clamped at 0.0893
  and 0.0888 rad against 0.0900 before, cola at 0.2623 against 0.2650 --
  which is the check that matters, because if the move had changed where
  the pads meet those bottles those numbers would have moved with it.
- One of the two opens is the best this project has recorded: the bell
  seated the full 21mm, 0.4mm off the cap's centre, with the bottle moving
  **0.1mm** while being pushed on, where previous successful runs moved
  3.7-19.5mm. Most of that is a bug the redesign flushed out rather than
  the layout itself -- see the next point.
- **`bartender_open` never put the counter into the planning scene**, on
  the grounds that `bartender_pour` publishes it. An open goal on a freshly
  started stack has no pour behind it, so there was no counter, and MoveIt
  plans through a worktop it has not been told about. It bit immediately on
  the new layout: arm B reached for the opener with its forearm 236mm BELOW
  the counter top and its gripper inside the worktop, and reported "fingers
  closed all the way without meeting anything 24.0mm wide", which is an
  honest description of a gripper jammed in a table. `bartender_open` now
  publishes the bar top itself, from the same numbers `bartender_pour` uses.
- Still true, and still the largest defect in the pour: the cola's
  place-and-release. It was thrown or left leaning in most runs before the
  redesign and it still is.
- `open_bottle` succeeded in **4 of 8** consecutive runs on the current
  31mm pads, against **6 of 8** on the 30mm pads they replaced. Each run is
  from a fresh simulator. At eight runs a side those two rates are not
  distinguishable, and the honest summary is that the pads got better and
  the pipeline did not.
- **The grip did improve, and that part is not ambiguous.** The bottle moved
  1.5 / 3.9 / 6.0 / 7.6mm while being pushed on, where the 30mm pads gave
  5.0 / 8.2 / 9.9 / 10.5 / 19.1 / 21.8mm. Every run beats the old mean and
  three of four beat its best case. Grip-caused failures went from 2 of 8 to
  1 of 8.
- What the remaining failures are, since they are no longer mostly the grip:
  two runs had the beer approach's first IK branch fail its descent, fall
  back to the next, and land 55mm off -- caught by the arrival check, and
  pre-existing (it happened once in the eight baseline runs too, and it does
  NOT scale with pad width: the 33mm pads saw none). One run passed the gate
  and published the detach, and the cap moved 3mm and stayed. One slipped
  45.8mm. So the approach and the shed now limit this, not the pads.
- On the runs that worked the bell seated 18.8-20.8mm of a possible 21,
  between 1.1 and 2.9mm off the cap's centre, and the freed cap ended up
  93mm to 319mm from the mouth.
- The pour was re-checked on the same pads, because it shares them and was
  tuned against the old ones: 1 of 2 runs poured a full whiskey-and-coke.
  The run that worked clamped the whiskey at 0.0900 rad and the cola at
  0.2650 -- the same figures recorded against the 30mm pads, which is the
  check that matters here: if the widening had moved where those two bottles
  are contacted, those numbers would have moved with it. The run that failed
  clamped the whiskey at 0.1715 instead and lost it mid-pour, which is the
  grip check doing its job on a bad landing rather than a change in the pad.
- The weak point is still the beer's grip, and widening the pads was not the
  fix -- it was worth doing, and it moved the number it was supposed to
  move, but two flat pads on a 38.7mm neck are two flat pads. Coulomb
  friction does not care about contact area, so a wider pad buys torque and
  no pull-out resistance at all. The mechanically right answer is a
  fingertip that hooks under the shoulder rather than pinching the neck --
  the `fingertip/` project next door is exactly that, and it is why it
  exists.
- `bartender_pour/pour_action_server.py`'s `WAYPOINTS_RAD` joint values are
  still placeholders, not tuned to the bottle/glass's actual poses in
  `bar_world.sdf` -- the bottle's pose is confirmed unchanged after a pour
  (gripper closes near it but doesn't actually contact/lift it). Jog the
  arm in RViz/MoveIt against the running sim to capture real values, and
  keep the SRDF's `home` group_state in sync (see
  `bartender_moveit_config/README.md`).
- No perception, no user interaction, no real fluid simulation -- see
  PLAN.md "Explicitly out of scope for this phase".
