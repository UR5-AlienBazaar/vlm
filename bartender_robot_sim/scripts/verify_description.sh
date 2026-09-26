#!/usr/bin/env bash
# Run once after installing ros-humble-ur-description / ur-simulation-gz /
# robotiq-description, and after building the workspace, to catch macro
# API mismatches before debugging them the hard way in Gazebo.
set -euo pipefail

echo "== ur_macro.xacro: xacro:macro name and params =="
grep -A2 'xacro:macro name="ur_robot"' "$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur_macro.xacro" \
  || echo "MISMATCH: macro name 'ur_robot' not found -- inspect ur_macro.xacro directly"

echo
echo "== robotiq_2f_85_macro.urdf.xacro: xacro:macro name and params =="
grep -A2 'xacro:macro name="robotiq' "$(ros2 pkg prefix robotiq_description)/share/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro" \
  || echo "MISMATCH: no robotiq_* macro found at expected path -- find the actual macro file/name"

echo
echo "== gripper actuated joint name(s) =="
grep -oE '<joint name="[^"]*"' "$(ros2 pkg prefix robotiq_description)/share/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro" \
  || true
echo "(compare against 'joint: ...' in bartender_description/config/controllers.yaml)"

echo
echo "== xacro renders without error =="
source /opt/ros/humble/setup.bash
xacro "$(ros2 pkg prefix bartender_description)/share/bartender_description/urdf/bartender.urdf.xacro" > /tmp/bartender.urdf
check_urdf /tmp/bartender.urdf
echo "OK: URDF is valid. See /tmp/bartender.urdf"
