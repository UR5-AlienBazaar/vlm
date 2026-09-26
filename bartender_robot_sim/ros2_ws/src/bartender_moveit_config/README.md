# bartender_moveit_config

Hand-written rather than MoveIt-Setup-Assistant-generated (the session that
authored this had no way to drive the Assistant's GUI interactively), but
verified working end-to-end in Gazebo Sim: `move_group` starts cleanly,
plans/executes joint-space goals for the `ur_manipulator` group, and drives
the `gripper` group's controller.

- `srdf/bartender.srdf` -- planning groups (`ur_manipulator`: base_link ->
  tool0 chain; `gripper`: the one actuated knuckle joint), group_states,
  and self-collision disables. The arm portion is adapted from
  ros-humble-ur-moveit-config's own SRDF (same chain, same joint names).
  Gripper self-collisions are disabled pairwise across all 9 links rather
  than only true kinematic-adjacency pairs -- see the comment at the top of
  the file for why that's a reasonable simplification here, not a shortcut
  that silently hides a real gap.
- `config/` -- kinematics.yaml, joint_limits.yaml, ompl_planning.yaml
  (arm portion copied from ur-moveit-config, group name already matched),
  moveit_controllers.yaml (mapped to this project's actual controller
  names: `ur_arm_controller`, `gripper_controller`).
- `launch/move_group.launch.py` -- same parameter set/structure as
  ur-moveit-config's own launch file, adapted for our single fixed robot
  (no per-instance UR driver args needed).

## Known issue

RViz's MotionPlanning display throws a `kinematics_solver_timeout` parameter
type error on startup (double vs. string) when launched via
`move_group.launch.py`'s `rviz_node`; `move_group` itself loads the same
`kinematics.yaml` without issue. `bartender_bringup/launch/bartender_sim.launch.py`
sets `launch_rviz:=false` by default because of this -- pass
`launch_rviz:=true` to re-enable and debug it if you need RViz visualization.

## If you retune the placeholder waypoints

`bartender_pour/pour_action_server.py`'s `WAYPOINTS_RAD['home']` should stay
in sync with the `home` group_state defined in `srdf/bartender.srdf`.
