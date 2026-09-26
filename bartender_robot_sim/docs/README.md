# Docs

Start here if you are new, or if you are building something that talks to
this robot.

| | |
|---|---|
| [WORKCELL.md](WORKCELL.md) | **Running the real robot**: the one-arm workcell, connecting the UR5e over Ethernet, and moving it. |
| [WORKCELL_API.md](WORKCELL_API.md) | **Workcell API**: picking bottles and making drinks over HTTP, for the programs that order them. |
| [ARCHITECTURE.md](ARCHITECTURE.md) | How the system is put together, the frame problem, why reachability is asymmetric, and where to add a new bottle / skill / front end. **Read this first.** |
| [CONTROL_API.md](CONTROL_API.md) | The proposed HTTP/JSON API for a VLM, planner or other non-ROS caller. **Proposal — not built yet.** |
| [ROADMAP.md](ROADMAP.md) | What works, what is blocking autonomy, phases, and ideas. |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to build and test, and the house rules — each one is here because breaking it cost a debugging session. |

Package-level detail lives with the package:

- [`bartender_teach/README.md`](../ros2_ws/src/bartender_teach/README.md) —
  the teach pendant, taught points, and recording pipelines
- [`bartender_moveit_config/README.md`](../ros2_ws/src/bartender_moveit_config/README.md)
  — why the MoveIt config is hand-written
- [`fingertip/README.md`](../fingertip/README.md) — the parametric fingertip
  generator

## If you are here to ...

**Drive the robot from a VLM or a script.** Read
[CONTROL_API.md](CONTROL_API.md). Note that it describes an API that does
not exist yet; today the entry points are the `PourDrink` and `OpenBottle`
ROS actions. The build order at the end of that document says what to
implement first and why.

**Add a bottle or move something on the bar.** "Where to add a new ..." in
[ARCHITECTURE.md](ARCHITECTURE.md), then the reachability section — a
position that looks fine on the counter is often not one any arm can
service.

**Teach a new sequence without writing code.**
[`bartender_teach/README.md`](../ros2_ws/src/bartender_teach/README.md),
section "Record mode".

**Work out why a run failed.** The error taxonomy in
[CONTROL_API.md](CONTROL_API.md) lists the failures this system actually
produces. "What is actually blocking autonomy" in
[ROADMAP.md](ROADMAP.md) covers the known-bad ones — check there before
assuming your change caused it.
