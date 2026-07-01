#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  map.sh  —  SmallBot interactive mapping                               ║
# ║                                                                         ║
# ║  1. Launches SLAM mapping (LiDAR + ESP32 + SLAM Toolbox + RViz)        ║
# ║  2. You drive the robot around with teleop in a second terminal        ║
# ║  3. Press Ctrl+C when done → map is auto-saved and overwrites old map  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

set -e

WORKSPACE=~/auto
MAP_DIR="${WORKSPACE}/maps"
MAP_NAME="smallbot_map"
MAP_PATH="${MAP_DIR}/${MAP_NAME}"

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'  # No Color

echo -e "${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║${BOLD}  SmallBot — Mapping Mode                                    ${NC}${CYAN}║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ── Source workspace ──────────────────────────────────────────────────────────
echo -e "${YELLOW}[1/4]${NC} Sourcing workspace..."
source /opt/ros/humble/setup.bash
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source "${WORKSPACE}/install/setup.bash"
else
    echo -e "${RED}ERROR: Workspace not built. Run:${NC}"
    echo "  cd ~/auto && colcon build --symlink-install"
    exit 1
fi

# ── Create maps directory ────────────────────────────────────────────────────
mkdir -p "${MAP_DIR}"

# ── Check for existing map ───────────────────────────────────────────────────
if [ -f "${MAP_PATH}.yaml" ]; then
    echo -e "${YELLOW}[!]${NC} Existing map found at ${MAP_PATH}.yaml"
    echo -e "    ${BOLD}It will be overwritten when you finish mapping.${NC}"
    echo ""
fi

# ── Launch mapping ───────────────────────────────────────────────────────────
echo -e "${YELLOW}[2/4]${NC} Starting SLAM mapping system..."
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ${BOLD}Open a SECOND terminal and run:${NC}${GREEN}                              ║${NC}"
echo -e "${GREEN}║                                                              ║${NC}"
echo -e "${GREEN}║  source ~/auto/install/setup.bash                            ║${NC}"
echo -e "${GREEN}║  ros2 run teleop_twist_keyboard teleop_twist_keyboard        ║${NC}"
echo -e "${GREEN}║                                                              ║${NC}"
echo -e "${GREEN}║  Drive the robot around to build the map.                    ║${NC}"
echo -e "${GREEN}║  Press ${BOLD}Ctrl+C HERE${NC}${GREEN} when done to save the map.                 ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Launch the SLAM system in background
ros2 launch slam_mapping full_system.launch.py &
LAUNCH_PID=$!

# ── Trap Ctrl+C to save map ──────────────────────────────────────────────────
cleanup_and_save() {
    echo ""
    echo -e "${YELLOW}[3/4]${NC} Stopping SLAM and saving map..."

    # Save the map (overwrite any existing)
    ros2 run nav2_map_server map_saver_cli \
        -f "${MAP_PATH}" \
        --ros-args -p save_map_timeout:=5000 2>/dev/null || true

    # Give map_saver a moment to finish writing
    sleep 2

    # Kill the SLAM launch
    kill ${LAUNCH_PID} 2>/dev/null || true
    wait ${LAUNCH_PID} 2>/dev/null || true

    # ── Verify map was saved ─────────────────────────────────────────────────
    echo ""
    if [ -f "${MAP_PATH}.yaml" ] && [ -f "${MAP_PATH}.pgm" ]; then
        PGM_SIZE=$(du -h "${MAP_PATH}.pgm" | cut -f1)
        echo -e "${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${GREEN}║  ${BOLD}✓ Map saved successfully!${NC}${GREEN}                                    ║${NC}"
        echo -e "${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
        echo ""
        echo -e "  ${BOLD}Map file:${NC}   ${MAP_PATH}.yaml"
        echo -e "  ${BOLD}Image file:${NC} ${MAP_PATH}.pgm  (${PGM_SIZE})"
        echo ""
        echo -e "  ${CYAN}To navigate on this map, run:${NC}"
        echo -e "    ${BOLD}./navigate.sh${NC}"
    else
        echo -e "${RED}╔══════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${RED}║  ✗ Map save FAILED!                                         ║${NC}"
        echo -e "${RED}╚══════════════════════════════════════════════════════════════╝${NC}"
        echo ""
        echo -e "  Try saving manually:"
        echo -e "    ros2 run nav2_map_server map_saver_cli -f ${MAP_PATH}"
    fi

    exit 0
}

trap cleanup_and_save SIGINT SIGTERM

# ── Wait for the launch process ──────────────────────────────────────────────
wait ${LAUNCH_PID}
