#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  navigate.sh  —  SmallBot autonomous navigation                        ║
# ║                                                                         ║
# ║  Loads a saved map and runs the full Nav2 stack:                        ║
# ║    • AMCL localization                                                  ║
# ║    • NavFn global planner (A*)                                          ║
# ║    • DWB local controller (dynamic obstacle avoidance)                  ║
# ║    • Path smoother                                                      ║
# ║    • Recovery behaviors (spin, backup, wait)                            ║
# ║    • Velocity smoother                                                  ║
# ║                                                                         ║
# ║  Set initial pose and send goals in RViz.                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

set -e

WORKSPACE=~/auto
MAP_DIR="${WORKSPACE}/maps"
MAP_NAME="smallbot_map"
MAP_PATH="${MAP_DIR}/${MAP_NAME}.yaml"

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║${BOLD}  SmallBot — Autonomous Navigation Mode                       ${NC}${CYAN}║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ── Source workspace ──────────────────────────────────────────────────────────
echo -e "${YELLOW}[1/3]${NC} Sourcing workspace..."
source /opt/ros/humble/setup.bash
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source "${WORKSPACE}/install/setup.bash"
else
    echo -e "${RED}ERROR: Workspace not built. Run:${NC}"
    echo "  cd ~/auto && colcon build --symlink-install"
    exit 1
fi

# ── Check map exists ─────────────────────────────────────────────────────────
echo -e "${YELLOW}[2/3]${NC} Checking for saved map..."
if [ ! -f "${MAP_PATH}" ]; then
    echo -e "${RED}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║  ✗ No saved map found!                                      ║${NC}"
    echo -e "${RED}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "  Expected map at: ${BOLD}${MAP_PATH}${NC}"
    echo -e "  Run ${BOLD}./map.sh${NC} first to create a map."
    exit 1
fi

PGM_FILE="${MAP_DIR}/${MAP_NAME}.pgm"
if [ -f "${PGM_FILE}" ]; then
    PGM_SIZE=$(du -h "${PGM_FILE}" | cut -f1)
    echo -e "  ${GREEN}✓${NC} Map found: ${MAP_PATH} (image: ${PGM_SIZE})"
else
    echo -e "  ${YELLOW}⚠${NC} Map YAML found but .pgm image is missing"
fi
echo ""

# ── Launch navigation ────────────────────────────────────────────────────────
echo -e "${YELLOW}[3/3]${NC} Launching Nav2 autonomous navigation..."
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ${BOLD}RViz will open shortly. Follow these steps:${NC}${GREEN}                 ║${NC}"
echo -e "${GREEN}║                                                              ║${NC}"
echo -e "${GREEN}║  1. Click '2D Pose Estimate' in the RViz toolbar             ║${NC}"
echo -e "${GREEN}║     → Click & drag on the map where the robot is NOW         ║${NC}"
echo -e "${GREEN}║     → AMCL particles will converge around the robot          ║${NC}"
echo -e "${GREEN}║                                                              ║${NC}"
echo -e "${GREEN}║  2. Click 'Nav2 Goal' in the RViz toolbar                    ║${NC}"
echo -e "${GREEN}║     → Click & drag on the map where you want the robot to GO ║${NC}"
echo -e "${GREEN}║     → The robot will plan a path and navigate autonomously    ║${NC}"
echo -e "${GREEN}║     → It will avoid dynamic obstacles in real-time            ║${NC}"
echo -e "${GREEN}║                                                              ║${NC}"
echo -e "${GREEN}║  Press Ctrl+C to stop navigation.                            ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Launch the Nav2 stack
ros2 launch slam_mapping navigation.launch.py \
    map:="${MAP_PATH}"
