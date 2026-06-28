#!/usr/bin/env bash
# Launch the LUCID camera ROS2 publisher.
#
# Points arena_api at the x64 Arena SDK that ships with ArenaView (the system
# ld.so.conf is wrongly aimed at the ARM64 SDK meant for the Jetson). Close
# ArenaView first — a GigE camera allows only one controlling app.
#
# Usage:  ./lucid_camera_start.sh [extra args passed to lucid_camera_node.py]
#   e.g.  ./lucid_camera_start.sh --binning 1 --width 1280 --fps 10
set -e

SDK="/home/yuval/Downloads/ArenaSDK_AVMP_v_1.0.8.60_Linux_x64/ArenaSDK_Linux_x64"
export LD_LIBRARY_PATH="$SDK/lib64:$SDK/GenICam/library/lib/Linux64_x64:$SDK/ffmpeg:$SDK/Metavision/lib:$LD_LIBRARY_PATH"
export GENICAM_GENTL64_PATH="$SDK/lib64"
export ROS_DOMAIN_ID=69
# arena_api reads its own version by shelling out to a bare `pip` — make sure that
# resolves to the venv's pip (where arena_api is registered), not the system one.
export PATH="$HOME/lucid_venv/bin:$PATH"

source /opt/ros/humble/setup.bash
source ~/projects/LearnROS2/ros2_ws/install/setup.bash 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python "$SCRIPT_DIR/lucid_camera_node.py" "$@"
