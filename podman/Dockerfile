# =============================================================================
#  ROS 2 Jazzy  ·  capstone_2026 arm robot
# =============================================================================
#
#  BUILD
#    podman build -t capstone2026 .
#
#  RUN  (default → rotating_test.py, X11 forwarded automatically)
#    podman run --rm -it \
#      -e DISPLAY=$DISPLAY \
#      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#      capstone2026
#
#  RUN  (pick a different launch file)
#    podman run --rm -it \
#      -e DISPLAY=$DISPLAY \
#      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#      capstone2026 --launch wasd_test.py
#
#  AVAILABLE LAUNCH FILES  (package: robot_launcher)
#    rotating_test.py   — sweep arm in circle + RViz  [DEFAULT]
#    wasd_test.py       — WASD keyboard teleop  + RViz
#
# =============================================================================

FROM osrf/ros:jazzy-desktop

# ---------------------------------------------------------------------------
# 1. System deps + X11 client libs
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        # X11 / OpenGL for RViz2
        libx11-6 \
        libxext6 \
        libgl1 \
        libglu1-mesa \
        x11-apps \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Install ros-jazzy-pinocchio
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. Clone only the src/ folder from capstone_2026  (main branch)
#    Repo layout:  capstone_2026/main/src/<packages>
# ---------------------------------------------------------------------------
ENV WS=/opt/ros2_ws
WORKDIR ${WS}

RUN git clone --filter=blob:none --no-checkout \
        https://github.com/sekyoung/capstone_2026.git /tmp/_repo \
    && cd /tmp/_repo \
    && git sparse-checkout init --cone \
    && git sparse-checkout set main/src \
    && git checkout main \
    # place packages directly at $WS/src/<pkg>
    && cp -r main/src ${WS}/src \
    && rm -rf /tmp/_repo

# ---------------------------------------------------------------------------
# 4. Resolve ROS package dependencies via rosdep
# ---------------------------------------------------------------------------
RUN . /opt/ros/jazzy/setup.sh \
    && rosdep update --rosdistro=jazzy \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro=jazzy \
        -y \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 5. colcon build
# ---------------------------------------------------------------------------
RUN . /opt/ros/jazzy/setup.sh \
    && cd ${WS} \
    && colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# ---------------------------------------------------------------------------
# 6. Convenience: auto-source on interactive shells
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash"     >> /root/.bashrc \
 && echo "source ${WS}/install/setup.bash"      >> /root/.bashrc \
 && echo "export DISPLAY=\${DISPLAY:-:0}"       >> /root/.bashrc

# ---------------------------------------------------------------------------
# 7. Entrypoint
# ---------------------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN  chmod +x /entrypoint.sh

# Default DISPLAY (overridden at runtime with -e DISPLAY=...)
ENV DISPLAY=:0

ENTRYPOINT ["/entrypoint.sh"]

# Default launch → rotating_test.py
# Override at runtime:  podman run ... capstone2026 --launch wasd_test.py
CMD ["--launch", "rotating_test.py"]
