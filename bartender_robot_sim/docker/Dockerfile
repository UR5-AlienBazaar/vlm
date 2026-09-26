FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        ros-humble-gz-ros2-control \
        ros-humble-moveit \
        ros-humble-robotiq-controllers \
        ros-humble-robotiq-description \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ur-description \
        ros-humble-ur-moveit-config \
        ros-humble-ur-robot-driver \
        ros-humble-ur-simulation-gz \
    && rm -rf /var/lib/apt/lists/*

# Bottle labels: PaddleOCR reads them offline, Gemini is the fallback. The models
# are fetched here so the server starts without network; the source check is off
# for the same reason. numpy<2 keeps ROS Humble's compiled bindings (cv_bridge) loadable.
ENV PADDLE_PDX_CACHE_HOME=/opt/models/paddlex \
    PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK=True
RUN pip3 install --no-cache-dir "numpy<2" paddlepaddle paddleocr google-genai \
    && python3 -c "from paddleocr import PaddleOCR; PaddleOCR( \
        use_doc_orientation_classify=False, use_doc_unwarping=False, \
        use_textline_orientation=False, enable_mkldnn=False)" \
    && chmod -R a+rwX /opt/models/paddlex
# ^ run_docker.sh runs as the host user, not the root that fetched the models.
# google-genai pulls in anyio, whose pytest plugin crashes Humble's apt pytest (colcon test included).
ENV PYTEST_ADDOPTS="-p no:anyio"

# Interactive shells automatically pick up an already-built workspace.
# The ROS image entrypoint sources /opt/ros/humble for every command.
RUN printf '%s\n' \
    'if [ -f /workspace/ros2_ws/install/setup.bash ]; then' \
    '  source /workspace/ros2_ws/install/setup.bash' \
    'fi' \
    > /etc/profile.d/bartender-workspace.sh \
    && printf '%s\n' \
    '' \
    '# Source the mounted bartender workspace when it has been built.' \
    '[ ! -f /workspace/ros2_ws/install/setup.bash ] || source /workspace/ros2_ws/install/setup.bash' \
    >> /etc/bash.bashrc

WORKDIR /workspace

CMD ["bash"]
