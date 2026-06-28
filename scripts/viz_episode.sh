#!/usr/bin/env bash
# viz_episode.sh — open a LeRobot dataset episode in the Rerun viewer.
# Shows the camera videos + observation.state / action timeseries (the motor
# recordings), so you can eyeball that an episode (e.g. a crash-recovered one)
# is really intact.
#
# Usage:  viz_episode.sh <episode_index> [dataset_name]
#   e.g.  viz_episode.sh 50 fuel_door
#         viz_episode.sh 95            # -> default dataset fuel_door
#
# Needs a display (run it on the laptop itself). The rerun viewer binary ships
# inside the lerobot venv but isn't on PATH by default — this handles that.
set -e
EP="${1:?usage: viz_episode.sh <episode_index> [dataset_name]}"
DATASET="${2:-fuel_door}"
ROOT="$HOME/lerobot_datasets/$DATASET"
RERUN_CLI="$HOME/lerobot_venv/lib/python3.10/site-packages/rerun_sdk/rerun_cli"

export PATH="$RERUN_CLI:$HOME/lerobot_venv/bin:$PATH"
export DISPLAY="${DISPLAY:-:0}"

exec "$HOME/lerobot_venv/bin/lerobot-dataset-viz" \
  --repo-id "ur10e/$DATASET" --root "$ROOT" \
  --episode-index "$EP" --mode local
