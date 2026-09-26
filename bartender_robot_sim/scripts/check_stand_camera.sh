#!/usr/bin/env bash
# Smoke-check the running sim's stand camera: Gazebo topics, bridged ROS topics, encoding and frame_id.
set -euo pipefail

DEPTH=/bartender/stand_camera/depth
INFO=/bartender/stand_camera/camera_info
FRAME=stand_camera_optical_frame
fail() { echo "FAIL: $*" >&2; exit 1; }

gz_topics="$(ign topic -l)"
grep -qx "$DEPTH" <<<"$gz_topics" || fail "Gazebo does not publish $DEPTH; stand-camera topics it does publish:
$(grep -i stand <<<"$gz_topics" || echo '  (none)')"
grep -qx "$INFO" <<<"$gz_topics" || fail "Gazebo does not publish $INFO; stand-camera topics it does publish:
$(grep -i stand <<<"$gz_topics" || echo '  (none)')"

info="$(timeout 10 ros2 topic echo --once "$INFO" sensor_msgs/msg/CameraInfo)" \
    || fail "no CameraInfo bridged on $INFO within 10s"
grep -q "frame_id: $FRAME" <<<"$info" \
    || fail "CameraInfo frame_id is not $FRAME: $(grep frame_id <<<"$info")"

encoding="$(timeout 10 ros2 topic echo --once --field encoding "$DEPTH" sensor_msgs/msg/Image)" \
    || fail "no depth image bridged on $DEPTH within 10s"
grep -q 32FC1 <<<"$encoding" || fail "depth encoding is not 32FC1: $encoding"

echo "OK: $DEPTH (32FC1) and $INFO (frame_id $FRAME) are live"
