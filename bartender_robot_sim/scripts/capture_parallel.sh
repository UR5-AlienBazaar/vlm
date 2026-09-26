#!/usr/bin/env bash
# Run N headless sims side by side, each capturing its own seed into OUT, then stop them.
#
#   scripts/capture_parallel.sh N SCENES OUT [capture_vlm_frames.py args...]
#   DISPLAY=:99 scripts/capture_parallel.sh 4 300 data/vlm_raw --pour-p 0.05
#
# One sim is CPU-bound at ~0.21x real time, so throughput comes from running
# several. Each gets its own container name, Gazebo partition and ROS domain;
# sim i captures seed SEED_BASE+i, so scene directories never collide.
# LAUNCH_ARGS is passed to the launch file, e.g. "headless_rendering:=true".
set -euo pipefail

N=$1 SCENES=$2 OUT=$3
shift 3
SEED_BASE=${SEED_BASE:-100}
LOG_DIR=${LOG_DIR:-/tmp}
READY_TIMEOUT_S=${READY_TIMEOUT_S:-600}
cd "$(dirname "$0")/.."

# Build once up front; the per-sim run_docker.sh calls then only hit the cache.
docker build --quiet --tag "${BARTENDER_DOCKER_IMAGE:-bartender-robot:humble}" \
    --file docker/Dockerfile docker >/dev/null

names=()
for i in $(seq 1 "$N"); do
    name=bartender-capture-$i
    names+=("$name")
    docker rm -f "$name" >/dev/null 2>&1 || true
    # shellcheck disable=SC2086  # LAUNCH_ARGS is a list of key:=value words
    BARTENDER_DOCKER_NAME=$name IGN_PARTITION=$name ROS_DOMAIN_ID=$((20 + i)) \
        setsid nohup docker/run_docker.sh ros2 launch bartender_bringup bartender_sim.launch.py \
        headless:=true ${LAUNCH_ARGS:-} > "$LOG_DIR/$name.sim.log" 2>&1 < /dev/null &
done

stop_all() { docker stop -t 5 "${names[@]}" >/dev/null 2>&1 || true; }
trap stop_all EXIT

for name in "${names[@]}"; do
    waited=0
    until grep -q 'You can start planning now' "$LOG_DIR/$name.sim.log" 2>/dev/null; do
        if (( waited >= READY_TIMEOUT_S )); then
            echo "$name not ready after ${READY_TIMEOUT_S}s, see $LOG_DIR/$name.sim.log" >&2
            exit 1
        fi
        sleep 5
        waited=$((waited + 5))
    done
    echo "$name ready"
done

pids=()
for i in $(seq 1 "$N"); do
    name=bartender-capture-$i
    docker exec "$name" bash -lc "source ros2_ws/install/setup.bash && \
        python3 scripts/capture_vlm_frames.py $OUT --scenes $SCENES --seed $((SEED_BASE + i)) $*" \
        > "$LOG_DIR/$name.capture.log" 2>&1 &
    pids+=($!)
done

status=0
for i in "${!pids[@]}"; do
    if wait "${pids[$i]}"; then
        echo "${names[$i]} done"
    else
        echo "${names[$i]} failed, see $LOG_DIR/${names[$i]}.capture.log" >&2
        status=1
    fi
done
exit $status
