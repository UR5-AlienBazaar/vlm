# Bartender Robot — Project Plan (Phase 1: Simulated Pouring)

> **SUPERSEDED — kept as a record of how Phase 1 went, not as a plan.**
>
> This document describes a single-armed robot whose pour waypoints were
> still placeholders. Since it was written the project gained a second arm,
> a beer-opening skill, a teach pendant, a rebuilt bar, and ~950 tests, and
> every "next step" below is long done.
>
> For what to do now, see **[docs/ROADMAP.md](docs/ROADMAP.md)**.
> For how the system is put together, **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**.
>
> It is worth keeping because the Progress section records what actually
> broke in phases 0-4 and how each was diagnosed -- the gz plugin search
> path gap, the nested-executor deadlock, the split
> `sim_gazebo`/`sim_ignition` flags.

## Goal
Get a UR5e + parallel gripper to reliably pick up a bottle and pour a measured
amount into a glass, fully in Gazebo simulation, driven by ROS2 nodes — before
touching a real robot or adding any user-facing interaction (voice/UI/orders
come later, out of scope for this phase).

## Decisions locked in
- **End effector:** Robotiq 2F-85 parallel gripper grasps the bottle by the
  neck, arm tilts the wrist to pour. (vs. fixed spout / welded bottle)
- **Motion stack:** MoveIt 2 for planning + `ros2_control` for execution.
  Chosen over hand-rolled joint trajectories for easier extension later
  (pick-and-place of multiple bottles, real-robot reuse, collision checking).
- **Simulator:** Gazebo Sim (Fortress, via `ros_gz`) — already installed on
  this machine (`ign`/`gz`, `libignition-gazebo6*`). Not Gazebo Classic.

## Environment (verified on this machine)
- Ubuntu 22.04, ROS2 Humble, `colcon` present.
- `ros_gz*` present (Gazebo Sim bridge). `gazebo-ros2-control` (classic) is
  also available if we ever need it, but we standardize on `gz_ros2_control`.
- Not yet installed, needed: `ros-humble-moveit`, `ros-humble-ur-description`,
  `ros-humble-ur-simulation-gz`, `ros-humble-ur-moveit-config`,
  `ros-humble-robotiq-description`, `ros-humble-robotiq-controllers`,
  `ros-humble-ros2-control`, `ros-humble-ros2-controllers`,
  `ros-humble-gz-ros2-control`.
- Existing assets: `models/Jack Daniel Bottle.obj` (needs converting into a
  proper Gazebo model: SDF + collision mesh + inertial properties + material).
- Not a git repo yet — initialize once workspace layout is in place.

## Workspace layout
```
bartender_robot/
├── PLAN.md
├── ros2_ws/
│   └── src/
│       ├── bartender_description/   # xacro/URDF: UR5e + Robotiq 2F-85 + bar scene
│       ├── bartender_gazebo/        # world SDF, launch files, spawn scripts
│       ├── bartender_moveit_config/ # generated MoveIt config (moveit_setup_assistant)
│       ├── bartender_bringup/       # top-level launch: sim + control + moveit
│       └── bartender_pour/          # ROS2 nodes: pour skill, glass/bottle TF, pour FSM
├── models/                          # Gazebo/SDF models (bottle, glass, bar counter)
└── code/                            # (existing, currently empty — fold into ros2_ws or repurpose for scripts/notebooks)
```
Recommend consolidating `code/` into the ROS2 workspace above rather than
keeping a separate loose folder, to avoid two sources of truth.

## Phased milestones

### Phase 0 — Environment & scaffolding
- `apt install` the packages listed above.
- `git init`, add `.gitignore` (build/, install/, log/).
- Create `ros2_ws` with the package skeletons above (`ros2 pkg create`).
- Sanity check: launch stock `ur_simulation_gz` demo to confirm UR5e spawns
  and moves in Gazebo Sim on this machine before writing any custom code.

### Phase 1 — Robot + gripper description
- Xacro-compose UR5e (`ur_description`) with Robotiq 2F-85
  (`robotiq_description`) at the tool flange.
- Add `ros2_control` tags + `gz_ros2_control` plugin so both arm joints and
  gripper fingers are controllable in sim.
- Verify: spawn in empty Gazebo world, move joints via
  `joint_trajectory_controller`, open/close gripper via its controller.

### Phase 2 — Bar scene
- Convert `Jack Daniel Bottle.obj` into a simulatable model: decimated
  collision mesh, inertial tags, SDF model with a `<link>`; add a simple
  cylindrical glass model and a bar-counter/table model.
- Build a Gazebo world placing robot, bottle stand, and glass at known,
  fixed poses (deferred: perception — for now poses are hardcoded/known TF
  frames, no vision needed to get first pour working).

### Phase 3 — MoveIt integration
- Generate MoveIt config for the UR5e+gripper combo (arm planning group +
  gripper group), tuned for the sim controllers from Phase 1.
- Verify in RViz: plan/execute arm motions, plan/execute gripper open/close,
  against the live Gazebo sim.

### Phase 4 — Pour skill (the core deliverable)
- New node/package `bartender_pour` implementing a simple state machine:
  1. Move to pre-grasp pose above bottle.
  2. Approach + close gripper (grasp bottle neck).
  3. Retreat, move to pre-pour pose above glass.
  4. Tilt wrist to pour angle, hold for a computed duration (time-based
     "measured pour" proxy — no real fluid sim), return to upright.
  5. Move back to bottle stand, release, return to home.
- Expose as a ROS2 action server (e.g. `PourDrink.action` with target glass
  ID / pour amount) so it's a clean interface for the later
  user-interaction layer to call — but no interaction layer built yet.
- Success criteria: action call reliably completes the pick-tilt-pour-return
  cycle in Gazebo across repeated runs without collisions or drops.

### Phase 5 — Validation & hardening
- Add basic collision checking margins, retry/error states (missed grasp,
  planning failure) reported back through the action's result/feedback.
- Log/replay a few full runs; tune pour angle & timing against a visual
  "how much appears to have poured" check (a full fluid simulation is out
  of scope for this phase).
- Document exact joint/tilt values and timing so the same skill can later
  be re-tuned for the real UR5e without a rewrite.

## Explicitly out of scope for this phase
- Real UR5e hardware bring-up (Phase 2 of the overall project).
- Any user interaction (voice, GUI, order queue, drink menu logic).
- Perception (camera-based bottle/glass detection) — poses are fixed/known.
- Real liquid/fluid simulation — pour is modeled as timed tilt, not volume.

## Progress

- **Phase 0 (done):** repo initialized, `ros2_ws/src` scaffolded with six
  packages, all building cleanly with `colcon build`. The apt install
  (README.md) has been run by the user; all target packages confirmed
  installed via `dpkg -l`.
- **Phase 1 (done, verified):** `bartender_description/urdf/bartender.urdf.xacro`
  composes `ur_macro.xacro` (UR5e) + `robotiq_2f_85_macro.urdf.xacro`
  (gripper) + `gz_ros2_control` plugin. Fixed against the real installed
  macro signatures (param names differed from what was guessed pre-install;
  also had to split `sim_gazebo`/`sim_ignition` -- passing both true emits
  two conflicting `<plugin>` tags) and a Gazebo plugin search path gap
  (`GZ_SIM_SYSTEM_PLUGIN_PATH` isn't set by ROS2 Humble's setup.bash).
  Verified live: full robot spawns in the bar world, `ros2 control
  list_hardware_interfaces` shows all 6 arm joints + the gripper joint, a
  `FollowJointTrajectory` goal moves the arm to the exact commanded
  positions, and a `GripperCommand` goal closes the gripper.
- **Phase 2 (done, verified):** as before -- bottle/glass/counter models,
  `bar_world.sdf`, loads cleanly and holds physics.
- **Phase 3 (done, verified):** `bartender_moveit_config` hand-written
  (SRDF, kinematics/joint_limits/ompl/controllers yaml, move_group launch)
  rather than Setup-Assistant-generated, adapted from ros-humble-ur-moveit-config's
  reference SRDF/launch/config for the arm portion, extended with the
  gripper group and its self-collision disables. Verified live: `move_group`
  starts cleanly, responds to services, and plans/executes joint-space
  goals against the real controllers. (Known cosmetic issue: RViz's
  MotionPlanning display throws a kinematics param type error on startup;
  RViz is off by default in bringup because of it -- see
  `bartender_moveit_config/README.md`.)
- **Phase 4 (done, verified):** `bartender_pour_interfaces/action/PourDrink.action`
  defined; `bartender_pour/pour_action_server.py` implements the
  pick -> tilt -> pour -> return state machine. Fixed a real deadlock bug
  (nested `rclpy.spin_until_future_complete()` calls grab a process-global
  executor that conflicts with the node's own `MultiThreadedExecutor`;
  switched to blocking on a `threading.Event` set by the future's
  done-callback instead). **Verified live end-to-end**: calling
  `/pour_drink` runs the full cycle through real feedback states
  (opening_gripper -> approaching_bottle -> grasping_bottle ->
  closing_gripper -> lifting_bottle -> moving_to_glass -> pouring ->
  returning_upright -> returning_bottle -> releasing_bottle ->
  returning_home) and returns `success: true`. Remaining gap: the
  `WAYPOINTS_RAD` joint values are still placeholders not tuned to the
  bottle/glass's real poses -- confirmed the bottle's pose is unchanged
  after a pour (gripper closes near it, doesn't actually contact/lift it).
- **Phase 5:** not started -- next real work is retuning waypoints against
  the actual bottle/glass poses so a pour physically moves the bottle.

## Next step
1. Jog the arm in RViz/MoveIt (or iterate via direct MoveGroup goals) to
   find real joint values that put the gripper around the bottle's actual
   neck position in `bar_world.sdf`, and around the glass for the pour
   pose. Update `WAYPOINTS_RAD` in `pour_action_server.py` and the `home`
   group_state in `bartender_moveit_config/srdf/bartender.srdf` to match.
2. Re-run the `pour_drink` action and confirm via `ign model -m
   jack_daniels_bottle -p` that the bottle's pose actually changes during
   the grasp/lift/pour/return sequence (not just that the action reports
   success).
3. Move on to Phase 5 hardening once a real physical pour is confirmed.
