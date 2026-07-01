#!/usr/bin/env bash
# train_curriculum.sh — the WHOLE skill curriculum as ONE continuous training.
#
# Everything lives in a SINGLE run folder (one continuous CSV, one continuous set of
# graphs, one TensorBoard record, episode/step counters carrying across stages, one
# final all-knowledge model). Stage transitions are internal curriculum events, not
# separate experiments:
#
#   2a  balance with actuated legs (no goal)
#   2b  drive to a goal point + station-keep
#   3a  re-balance with the wide SQUAT-joint range (ducking needs 0.7 rad knees; the
#       Stage-2 policy saturates the action clip, so it must briefly re-adapt)
#   3b  obstacle course: dodge pillars, SQUAT under the low table
#   4   stairs + rough terrain
#
# Each stage trains until LEARNED (--advance-* rolling thresholds; --total-timesteps is
# only a safety cap), then the next stage warm-starts from the shared model_best.pth
# (weight surgery when the observation grows).
#
# Layout: the MAIN folder holds the continuum (continuous metrics/graphs/_overview.png,
# model_best/latest/final at the root = the all-knowledge models); each stage also gets
# a NAMED SUBFOLDER inside it with that stage's best snapshot:
#   curriculum1/
#   ├── model_best.pth, model_final.pth, ...   <- the continuum / all-knowledge models
#   ├── metrics/ (ONE continuous CSV + graphs + TB across all stages), videos/, train.log
#   ├── s2a_balance_legs/model_best.pth        <- per-stage snapshots
#   ├── s2b_goal_nav/model_best.pth  ... etc.
#
# Reproducible on any machine: paths derive from this script's location. Requirements:
# python with mujoco+gymnasium+torch+numpy+matplotlib.
#
# Usage:
#   [PYTHON=/path/to/python] [NUM_ENVS=24] [RUN_DIR=...] ./train_curriculum.sh [start-stage]
#     start-stage: 2a | 2b | 3a | 3b | 4     (default 2a = full curriculum)
set -u
SCR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCR/../../.." && pwd)"
RES="$REPO/training_results"
RUN="${RUN_DIR:-$RES/curriculum1}"
PY="${PYTHON:-python3}"
NE="${NUM_ENVS:-24}"
export PYTHONPATH="$SCR:${PYTHONPATH:-}"
cd "$SCR" || exit 1
mkdir -p "$RUN"

idx() { case "$1" in 2a) echo 0;; 2b) echo 1;; 3a) echo 2;; 3b) echo 3;; 4) echo 4;;
        *) echo "unknown stage '$1' (use 2a|2b|3a|3b|4)" >&2; exit 2;; esac; }
START_I="$(idx "${1:-2a}")" || exit 2
FIRST_RUN=1

train() {  # train <stage> <stage-name> <env> <steps> [extra ppo_parallel args...]
  local stage="$1" name="$2" env="$3" steps="$4"; shift 4
  if [ "$(idx "$stage")" -lt "$START_I" ]; then
    echo "[curriculum] skip stage $stage (already trained)"; return 0
  fi
  local warm=()
  if [ "$FIRST_RUN" -eq 1 ] && [ "$(idx "$stage")" -eq 0 ]; then
    warm=()                                   # very first stage: from scratch
  else
    warm=(--load "$RUN/model_best.pth")       # continue the same brain
  fi
  FIRST_RUN=0
  echo "[curriculum] ===== stage $stage ($name): env=$env (continuing in $RUN) ====="
  "$PY" -u ppo_parallel.py --env "$env" --num-envs "$NE" --n-steps 128 \
      --total-timesteps "$steps" --checkpoint-every 200 \
      --session-id curriculum --run-dir "$RUN" "${warm[@]}" "$@" || exit 1
  mkdir -p "$RUN/s${stage}_${name}"           # per-stage snapshot folder INSIDE the run
  cp -f "$RUN/model_best.pth" "$RUN/s${stage}_${name}/model_best.pth"
}

train 2a balance_legs   mujoco_nav_bal      5000000 --advance-ep-len 900
train 2b goal_nav       mujoco_nav          6000000 --advance-ep-rew 2500
train 3a wide_squat     mujoco_obstacle_bal 4000000 --advance-ep-rew 2000 --load-leg-old-scale 0.25
train 3b obstacles_duck mujoco_obstacle     8000000 --advance-ep-rew 1800
train 4  terrain        mujoco_terrain      8000000

echo "[curriculum] ALL STAGES DONE — final all-knowledge model: $RUN/model_final.pth"
