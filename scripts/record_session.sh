#!/usr/bin/env bash
# record_session.sh — one-terminal data-collection launcher for the UR10e teleop rig.
#
# Brings up BOTH cameras and the LeRobot recorder from a single terminal, and
# verifies every stream is actually publishing before recording (so you never
# get a silent "missing: ['depth']" 0-frame episode again):
#   1. RealSense D435i  — launched WITH aligned depth enabled
#   2. LUCID Triton     — scene camera (lucid_camera_start.sh)
#   3. episode_recorder — 2-camera, fps 10, into the 2-cam dataset
# Cameras run in the background and are cleaned up on Ctrl-C.
#
# The robot driver (ur10e_start.sh) and the teleop node still run in their own
# terminals — this script is just the "cameras + recorder" half.
#
# Usage:   record_session.sh "<task description>" [dataset_name] [extra recorder args]
#   e.g.   record_session.sh "open the fuel door" fuel_door
#          record_session.sh "plug in the charger" charge_plug --voice-type male3
#          record_session.sh "pop the door"            # -> default folder ur10e_teleop_2cam
#
# dataset_name is the FOLDER under ~/lerobot_datasets/. Reusing a name APPENDS more
# episodes to that dataset; a new name starts a fresh one. The "<task description>"
# is the language label stored on every episode (used by language-conditioned policies).
set -e

TASK="${1:?usage: record_session.sh \"<task>\" [dataset_name] [extra recorder args]}"; shift
# optional 2nd positional = dataset/folder name (anything not starting with --)
DATASET="ur10e_teleop_2cam"
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then DATASET="$1"; shift; fi

ROOT="$HOME/lerobot_datasets/$DATASET"
REPO="ur10e/$DATASET"
LUCID_TOPIC="/lucid/image_raw"
COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
SCRIPTS="$HOME/projects/LearnROS2/ros2_ws/src/Quest2ROS2/scripts"
LOG=/tmp/record_session; mkdir -p "$LOG"

source /opt/ros/humble/setup.bash
source "$HOME/projects/LearnROS2/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=69

echo "[record_session] task=\"$TASK\""
echo "[record_session] dataset=$ROOT"
[ -d "$ROOT/meta" ] && echo "[record_session] (folder exists — APPENDING episodes)" \
                    || echo "[record_session] (new folder — fresh dataset)"

PIDS=()
cleanup() {
  echo; echo "[record_session] stopping cameras…"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_topic() {  # $1=topic  $2=label  — wait until it has >=1 publisher
  echo -n "[record_session] waiting for $2 ($1) … "
  for _ in $(seq 1 60); do
    n=$(ros2 topic info "$1" 2>/dev/null | sed -n 's/.*Publisher count: //p')
    [ "${n:-0}" -ge 1 ] && { echo "up"; return 0; }
    sleep 1
  done
  echo "TIMEOUT — not publishing"; return 1
}

echo "[record_session] launching RealSense D435i (aligned depth ON)…"
# 640x480 wrist (plenty for policy learning; 720p just costs RAM/encode). Aligned
# depth follows the color resolution, so it shrinks to 640x480 too.
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
     rgb_camera.color_profile:=640x480x30 >"$LOG/d435i.log" 2>&1 &
PIDS+=($!)

echo "[record_session] launching LUCID camera…"
"$SCRIPTS/lucid_camera_start.sh" >"$LOG/lucid.log" 2>&1 &
PIDS+=($!)

wait_topic "$COLOR_TOPIC" "D435i color"        || { echo "  see $LOG/d435i.log"; exit 1; }
wait_topic "$DEPTH_TOPIC" "D435i aligned depth" || { echo "  see $LOG/d435i.log"; exit 1; }
wait_topic "$LUCID_TOPIC" "LUCID scene"         || { echo "  see $LOG/lucid.log"; exit 1; }

echo "[record_session] all streams live. Starting recorder."
echo "[record_session]   B=start/stop episode, X=discard, Y=e-stop."
~/lerobot_venv/bin/python "$SCRIPTS/episode_recorder.py" \
  --task "$TASK" --root "$ROOT" --repo-id "$REPO" \
  --lucid-topic "$LUCID_TOPIC" --fps 10 "$@"
