#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="${BARTENDER_DOCKER_IMAGE:-bartender-robot:humble}"
CONTAINER_NAME="${BARTENDER_DOCKER_NAME:-bartender-robot}"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
CONTAINER_RUNTIME_DIR="/tmp/runtime-${HOST_UID}"
CONTAINER_HOME_DIR="${CONTAINER_RUNTIME_DIR}/home"
GPU_REQUEST="${BARTENDER_DOCKER_GPUS:-}"

# Prefer NVIDIA Container Toolkit when both the host driver and Docker
# runtime are available. BARTENDER_DOCKER_GPUS remains an explicit override.
nvidia_host=false
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    nvidia_host=true
    docker_runtimes="$(docker info --format '{{json .Runtimes}}' 2>/dev/null || true)"
    if [[ -z "${GPU_REQUEST}" && "${docker_runtimes}" == *nvidia* ]]; then
        GPU_REQUEST=all
    fi
fi

docker build \
    --tag "${IMAGE_NAME}" \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

# A numeric UID without a passwd entry gives Ignition processes inconsistent
# default partitions. Use one explicit partition for this project.
docker_args=(
    --rm
    --name "${CONTAINER_NAME}"
    --network host
    --ipc host
    --user "${HOST_UID}:${HOST_GID}"
    --env "HOME=${CONTAINER_HOME_DIR}"
    --env "DISPLAY=${DISPLAY:-}"
    --env "IGN_PARTITION=${IGN_PARTITION:-bartender-robot-${HOST_UID}}"
    # With --network host, sims running side by side share DDS unless each
    # gets its own domain; they would then drive each other's arms.
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    --env "IGN_GAZEBO_RESOURCE_PATH=/workspace/models"
    --env "QT_X11_NO_MITSHM=1"
    --env "XDG_RUNTIME_DIR=${CONTAINER_RUNTIME_DIR}"
    --tmpfs "${CONTAINER_RUNTIME_DIR}:rw,nosuid,nodev,mode=0700,uid=${HOST_UID},gid=${HOST_GID}"
    --volume "${PROJECT_DIR}:/workspace"
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --workdir /workspace
)

# Secrets such as GEMINI_API_KEY stay out of the image: they come from the
# git-ignored .env at run time (KEY=value lines, no quotes, no `export`).
if [[ -f "${PROJECT_DIR}/.env" ]]; then
    docker_args+=(--env-file "${PROJECT_DIR}/.env")
fi

# Forward the current X11 authentication cookie without opening the X server
# to every local client (as `xhost +` would do).
xauthority_path="${XAUTHORITY:-${HOME}/.Xauthority}"
if [[ -n "${DISPLAY:-}" && -r "${xauthority_path}" ]]; then
    docker_args+=(
        --env "XAUTHORITY=/tmp/.docker.xauth"
        --volume "${xauthority_path}:/tmp/.docker.xauth:ro"
    )
elif [[ -n "${DISPLAY:-}" ]]; then
    echo "Warning: no readable Xauthority file at ${xauthority_path}; X11 may reject the connection." >&2
else
    echo "Warning: DISPLAY is not set; X11 GUI applications will not be available." >&2
fi

# Give Gazebo/RViz direct-rendering access on Intel/AMD systems when present.
# Proprietary NVIDIA uses the Container Toolkit instead of a raw DRI mount.
if [[ -n "${GPU_REQUEST}" ]]; then
    docker_args+=(
        --gpus "${GPU_REQUEST}"
        --env "NVIDIA_DRIVER_CAPABILITIES=all"
    )
elif [[ "${nvidia_host}" == false && -d /dev/dri ]]; then
    docker_args+=(--volume /dev/dri:/dev/dri)
    while IFS= read -r device_group; do
        docker_args+=(--group-add "${device_group}")
    done < <(find /dev/dri -maxdepth 1 -type c -printf '%G\n' | sort -u)
else
    # Keep Qt/Gazebo usable on hosts without a render device. This is slower,
    # but avoids accidentally loading a host GPU driver that cannot work in
    # the container (for example nouveau without a corresponding /dev/dri).
    docker_args+=(--env "LIBGL_ALWAYS_SOFTWARE=1")
fi

if [[ -t 0 && -t 1 ]]; then
    docker_args+=(--interactive --tty)
else
    docker_args+=(--interactive)
fi

if (( $# == 0 )); then
    command=(bash -lc 'mkdir -p "$HOME"; exec bash')
else
    # Source a previously built workspace for non-interactive commands too.
    command=(
        bash -lc
        'mkdir -p "$HOME"; [ ! -f ros2_ws/install/setup.bash ] || source ros2_ws/install/setup.bash; exec "$@"'
        bash "$@"
    )
fi

docker run "${docker_args[@]}" "${IMAGE_NAME}" "${command[@]}"
