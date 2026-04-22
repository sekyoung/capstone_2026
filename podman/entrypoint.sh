#!/bin/bash
# =============================================================================
#  entrypoint.sh  ·  capstone_2026 container entry point
#
#  OPTIONS
#    --launch  <file.py>    Launch file to run          (default: rotating_test.py)
#    --package <pkg_name>   ROS 2 package owning it     (default: robot_launcher)
#    --list                 Print available launch files and exit
#    --help                 Print this help and exit
#
#  EXAMPLES
#    # Default (rotating sweep + RViz)
#    podman run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw capstone2026
#
#    # WASD teleop
#    podman run ... capstone2026 --launch wasd_test.py
#
#    # Custom package / launch file
#    podman run ... capstone2026 --launch my_file.launch.py --package my_pkg
# =============================================================================
set -e

# ── Source ROS 2 + workspace ────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source /opt/ros2_ws/install/setup.bash

# ── Defaults ────────────────────────────────────────────────────────────────
LAUNCH_FILE="rotating_test.py"
PACKAGE="robot_launcher"

# ── Argument parsing ─────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --launch)
            LAUNCH_FILE="$2"
            shift 2
            ;;
        --package)
            PACKAGE="$2"
            shift 2
            ;;
        --list)
            echo ""
            echo "Available launch files in package 'robot_launcher':"
            echo "  rotating_test.py   — Sweep arm in circle + RViz  [DEFAULT]"
            echo "  wasd_test.py       — WASD keyboard teleop + RViz"
            echo ""
            echo "Usage:  podman run ... capstone2026 --launch <file.py>"
            exit 0
            ;;
        --help|-h)
            cat <<'EOF'

capstone_2026 container — ROS 2 Jazzy arm robot

USAGE
  podman run [podman-opts] capstone2026 [OPTIONS]

OPTIONS
  --launch  <file.py>    Launch file name  (default: rotating_test.py)
  --package <pkg_name>   Package that owns the launch file (default: robot_launcher)
  --list                 List known launch files and exit
  --help                 Show this help and exit

X11 FORWARDING (required for RViz2)
  podman run --rm -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    capstone2026

EXAMPLES
  # Default sweep demo
  podman run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw capstone2026

  # WASD teleop
  podman run ... capstone2026 --launch wasd_test.py

  # Custom launch file in a different package
  podman run ... capstone2026 --launch foo.launch.py --package my_pkg

EOF
            exit 0
            ;;
        *)
            echo "[entrypoint] ERROR: Unknown argument '$1'"
            echo "             Run with --help for usage."
            exit 1
            ;;
    esac
done

# ── Sanity-check: verify the launch file actually exists ────────────────────
LAUNCH_PATH=$(ros2 pkg prefix "${PACKAGE}" 2>/dev/null)/share/${PACKAGE}/launch/${LAUNCH_FILE}

if [[ ! -f "${LAUNCH_PATH}" ]]; then
    echo ""
    echo "[entrypoint] ERROR: Launch file not found:"
    echo "             ${LAUNCH_PATH}"
    echo ""
    echo "  Available launch files in package '${PACKAGE}':"
    find "$(ros2 pkg prefix "${PACKAGE}" 2>/dev/null)/share/${PACKAGE}/launch/" \
         -name "*.py" -o -name "*.xml" -o -name "*.yaml" 2>/dev/null \
        | sed 's/.*\//    /'
    echo ""
    echo "  Run with --list for a summary, or --help for full usage."
    exit 1
fi

# ── Banner ───────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║     capstone_2026  ·  ROS 2 Jazzy               ║"
echo "╠══════════════════════════════════════════════════╣"
printf  "║  package : %-37s║\n" "${PACKAGE}"
printf  "║  launch  : %-37s║\n" "${LAUNCH_FILE}"
printf  "║  DISPLAY : %-37s║\n" "${DISPLAY}"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── Launch ───────────────────────────────────────────────────────────────────
exec ros2 launch "${PACKAGE}" "${LAUNCH_FILE}"
