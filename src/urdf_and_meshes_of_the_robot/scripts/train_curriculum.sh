#!/usr/bin/env bash
# train_curriculum.sh — the WHOLE skill curriculum as ONE autonomous continuum.
#
# Stages advance automatically: each trains until LEARNED (rolling episode metrics
# cross its --advance-* threshold, with --total-timesteps as the safety cap), then the
# next stage warm-starts from its best checkpoint:
#
#   2a  balance with actuated legs (no goal)        -> training_results/nav3a_balleg
#   2b  drive to a goal point + station-keep        -> training_results/nav3b_goal
#   3a  re-balance at the wide leg range (ducking   -> training_results/nav4a_widelegs
#       needs 0.7 rad legs; Stage-2 saturates the clip, so this brief re-adaptation
#       stage is required -- see CLAUDE.md / commit history)
#   3b  obstacle course: dodge pillars, DUCK under  -> training_results/nav4_obstacle
#       the low table
#   4   stairs + rough terrain                      -> training_results/nav5_terrain
#
# Reproducible on any machine: paths derive from this script's location; no user-
# specific paths. Requirements: python with mujoco+gymnasium+torch+numpy+matplotlib
# (see repo README / requirements).
#
# Usage:
#   [PYTHON=/path/to/python] [NUM_ENVS=24] ./train_curriculum.sh [start-stage]
#     start-stage: 2a | 2b | 3a | 3b | 4     (default 2a = full curriculum)
# Examples:
#   ./train_curriculum.sh                        # everything from scratch
#   PYTHON=~/anaconda3/envs/rl/bin/python ./train_curriculum.sh 3a   # resume at 3a
set -u
SCR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCR/../../.." && pwd)"
RES="$REPO/training_results"
PY="${PYTHON:-python3}"
NE="${NUM_ENVS:-24}"
export PYTHONPATH="$SCR:${PYTHONPATH:-}"
cd "$SCR" || exit 1

idx() { case "$1" in 2a) echo 0;; 2b) echo 1;; 3a) echo 2;; 3b) echo 3;; 4) echo 4;;
        *) echo "unknown stage '$1' (use 2a|2b|3a|3b|4)" >&2; exit 2;; esac; }
START_I="$(idx "${1:-2a}")" || exit 2

train() {  # train <stage> <env> <run-name> <steps> [extra ppo_parallel args...]
  local stage="$1" env="$2" run="$3" steps="$4"; shift 4
  if [ "$(idx "$stage")" -lt "$START_I" ]; then
    echo "[curriculum] skip stage $stage (starting at ${1:-})"; return 0
  fi
  echo "[curriculum] ===== stage $stage: env=$env -> $RES/$run ====="
  "$PY" -u ppo_parallel.py --env "$env" --num-envs "$NE" --n-steps 128 \
      --total-timesteps "$steps" --checkpoint-every 100 \
      --session-id "s$stage" --run-dir "$RES/$run" "$@" || exit 1
}

train 2a mujoco_nav_bal      nav3a_balleg   5000000 --advance-ep-len 900
train 2b mujoco_nav          nav3b_goal     6000000 --advance-ep-rew 2500 \
      --load "$RES/nav3a_balleg/model_best.pth"
train 3a mujoco_obstacle_bal nav4a_widelegs 4000000 --advance-ep-rew 2000 \
      --load "$RES/nav3b_goal/model_best.pth" --load-leg-old-scale 0.25
train 3b mujoco_obstacle     nav4_obstacle  8000000 --advance-ep-rew 1800 \
      --load "$RES/nav4a_widelegs/model_best.pth"
train 4  mujoco_terrain      nav5_terrain   8000000 \
      --load "$RES/nav4_obstacle/model_best.pth"

echo "[curriculum] ALL STAGES DONE"
