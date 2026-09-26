#!/usr/bin/env bash
# Start the workcell twin (mock UR5e + Gazebo twin, headless) for VLM captures,
# with the live camera viewer on http://localhost:${VIEWER_PORT:-8090}.
#
#   scripts/workcell_vlm_twin.sh                 # start, wait until ready
#   docker exec vlm-twin bash -lc "source ros2_ws/install/setup.bash && \
#       python3 scripts/capture_workcell_scenes.py data/workcell_raw/run1 --scenes 50"
#
# Not run_docker.sh: that rebuilds the image first and uses host networking,
# which on Docker Desktop is the VM's network, so the viewer port would not
# reach Windows. Every ROS node here lives in this one container, so DDS
# does not need the host network.
set -euo pipefail
cd "$(dirname "$0")/.."
NAME=${TWIN_NAME:-vlm-twin}
IMAGE=${BARTENDER_DOCKER_IMAGE:-bartender-robot:humble}
PORT=${VIEWER_PORT:-8090}

docker rm -f "$NAME" >/dev/null 2>&1 || true
gpu=()
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    gpu=(--gpus "${GPUS:-all}" --env NVIDIA_DRIVER_CAPABILITIES=all)
fi
docker run -d --name "$NAME" "${gpu[@]}" -p "127.0.0.1:$PORT:$PORT" \
    --env IGN_PARTITION="$NAME" --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-57}" \
    --user "$(id -u):$(id -g)" --env HOME=/tmp \
    -v "$PWD:/workspace" -w /workspace "$IMAGE" \
    bash -c 'source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && \
        exec ros2 launch bartender_bringup workcell_twin.launch.py \
        use_fake_hardware:=true headless:=true headless_rendering:=true' >/dev/null

for _ in $(seq 120); do
    docker logs "$NAME" 2>&1 | grep -q 'mirroring /joint_states' && break
    sleep 2
done
docker logs "$NAME" 2>&1 | grep -q 'mirroring /joint_states' || { echo "$NAME not ready" >&2; exit 1; }
docker exec -d "$NAME" bash -c "source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash && \
    python3 scripts/workcell_viewer.py --port $PORT > /tmp/viewer.log 2>&1"
echo "$NAME ready, viewer on http://localhost:$PORT"
