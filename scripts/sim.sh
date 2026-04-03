#!/usr/bin/env bash
# Launch full Gazebo SITL simulation environment.
# Opens 4 terminal windows: Gazebo, ArduPilot SITL, MAVROS, and drone_node.
#
# Prerequisites:
#   - ArduPilot SITL installed (sim_vehicle.py on PATH)
#   - ardupilot_gazebo built in ~/gz_ws/src/ardupilot_gazebo/build
#   - ROS 2 Humble + MAVROS installed
#   - Drone package built (colcon build)
#
# Usage:
#   ./scripts/sim.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

GZ_PLUGIN_PATH="${HOME}/gz_ws/src/ardupilot_gazebo/build"
GZ_MODEL_PATH="${HOME}/gz_ws/src/ardupilot_gazebo/models"
GZ_WORLD_PATH="${HOME}/gz_ws/src/ardupilot_gazebo/worlds"

echo "Starting Gazebo SITL simulation..."

# Terminal 1: Gazebo
gnome-terminal --title="Gazebo" -- bash -c "
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_PLUGIN_PATH}:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=${GZ_MODEL_PATH}:${GZ_WORLD_PATH}:\${GZ_SIM_RESOURCE_PATH:-}
gz sim -v4 -r iris_runway.sdf
exec bash" &

sleep 2

# Terminal 2: ArduPilot SITL
gnome-terminal --title="ArduPilot SITL" -- bash -c "
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
exec bash" &

sleep 10

# Terminal 3: MAVROS
gnome-terminal --title="MAVROS" -- bash -c "
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
exec bash" &

sleep 3

# Terminal 4: Drone node
gnome-terminal --title="Drone" -- bash -c "
source /opt/ros/humble/setup.bash
cd ${PROJECT_DIR}/..
source install/setup.bash
ros2 launch drone drone.launch.py
exec bash" &

echo "All terminals launched. Close them individually to stop."
wait
