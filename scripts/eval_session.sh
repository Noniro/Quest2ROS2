#!/usr/bin/env bash
# eval_session.sh — autonomous policy-evaluation launcher (the "play" counterpart
# to record_session.sh). Brings up BOTH cameras, verifies they publish, then runs
# policy_eval.py: the trained policy drives the UR10e for N trials and you score
# each one. Cameras are cleaned up on Ctrl-C.
#
# Same robot bring-up as recording is required FIRST, in its own terminal:
#   T1: ur10e_start.sh         (driver + forward_position_controller)
#   (NO teleop node and NO Quest headset — the policy drives the robot during eval,
#    and you score each trial from the laptop KEYBOARD. Keep a hand on the pendant e-stop.)
#
# Usage:  eval_session.sh "<task>" [dataset_label] [extra policy_eval args]
#   e.g.  eval_session.sh "open the fuel door" fuel_door --policy-host http://10.0.0.78:9000
#         eval_session.sh "open the fuel door" fuel_door --trials 10
#
# ⚠️ Confirm with your coworker HOW the model is served and pass --policy-host
#    (remote, default) or --serve local --checkpoint <path> (needs a GPU).
set -e

TASK="${1:?usage: eval_session.sh \"<task>\" [dataset_label] [extra args]}"; shift
DATASET="fuel_door"
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then DATASET="$1"; shift; fi

LUCID_TOPIC="/lucid/image_raw"
COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
SCRIPTS="$HOME/projects/LearnROS2/ros2_ws/src/Quest2ROS2/scripts"
LOG=/tmp/eval_session; mkdir -p "$LOG"

source /opt/ros/humble/setup.bash
source "$HOME/projects/LearnROS2/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=69

echo "[eval_session] task=\"$TASK\"  dataset=$DATASET"

PIDS=()
cleanup() {
  echo; echo "[eval_session] stopping cameras…"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_topic() {  # $1=topic $2=label
  echo -n "[eval_session] waiting for $2 ($1) … "
  for _ in $(seq 1 60); do
    n=$(ros2 topic info "$1" 2>/dev/null | sed -n 's/.*Publisher count: //p')
    [ "${n:-0}" -ge 1 ] && { echo "up"; return 0; }
    sleep 1
  done
  echo "TIMEOUT — not publishing"; return 1
}

echo "[eval_session] launching RealSense D435i…"
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true \
     rgb_camera.color_profile:=640x480x30 >"$LOG/d435i.log" 2>&1 &
PIDS+=($!)

echo "[eval_session] launching LUCID camera…"
"$SCRIPTS/lucid_camera_start.sh" >"$LOG/lucid.log" 2>&1 &
PIDS+=($!)

wait_topic "$COLOR_TOPIC" "D435i color" || { echo "  see $LOG/d435i.log"; exit 1; }
wait_topic "$LUCID_TOPIC" "LUCID scene" || { echo "  see $LOG/lucid.log"; exit 1; }

echo "[eval_session] cameras live. Starting policy evaluation."
echo "[eval_session]   keyboard: r=ready/go  s=stop run  t=success  f=fail  (letter + Enter)"
echo "[eval_session]   KEEP A HAND ON THE PENDANT E-STOP — the policy drives the arm."
# eval runs in ~/eval_venv (openpi-client pins numpy<2; lerobot_venv stays numpy 2).
PY="${EVAL_PYTHON:-$HOME/eval_venv/bin/python}"
"$PY" "$SCRIPTS/policy_eval.py" \
  --task "$TASK" --dataset "$DATASET" "$@"
