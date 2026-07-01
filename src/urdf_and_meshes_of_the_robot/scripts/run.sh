#!/usr/bin/env bash
# run.sh — one entry point for the mybot wheel-legged balance project.
#
#   ./run.sh selftest          # validate the from-scratch PPO on Pendulum-v1 (no robot)
#   ./run.sh sim               # launch the headless Gazebo sim + ros2_control
#   ./run.sh train             # run the PPO trainer (needs `sim` running in another shell)
#   ./run.sh record            # real-Gazebo periodic recorder (needs `sim` running)
#   ./run.sh stop              # stop our sim + trainer (ONLY ours, never another sim)
#
# Assumes ROS 2 Humble + the package built in a colcon workspace and sourced:
#   cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
set -e
PKG=urdf_and_meshes_of_the_robot
SHARE=$(ros2 pkg prefix "$PKG")/share/$PKG
SCRIPTS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PYTHONPATH="$SCRIPTS:${PYTHONPATH:-}"
PY="${PY:-python3}"                                   # set PY=path to a torch+gymnasium python
RUN_DIR="${RUN_DIR:-$PWD/training_results/run1}"

case "${1:-help}" in
  selftest)
    "$PY" "$SCRIPTS/ppo_balance.py" --env Pendulum-v1 --total-timesteps 100000 \
        --run-dir "$RUN_DIR" ;;
  sim)
    # Headless sim for training + recording. The periodic recorder spawns a camera into
    # gzserver, which needs an X display to render it. WSLg's :0 is not reliably present
    # in headless shells, so bring up a self-contained Xvfb on :99 (no-op if one is up).
    if ! xdpyinfo >/dev/null 2>&1; then
      pgrep -f "Xvfb :99" >/dev/null 2>&1 || { Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp >/tmp/xvfb_mybot.log 2>&1 & sleep 2; }
      export DISPLAY=:99
    fi
    ros2 launch "$PKG" balance.launch.py gui:=false world:="${WORLD:-world1.world}" ;;
  train)
    "$PY" "$SCRIPTS/ppo_balance.py" --env balance --device "${DEVICE:-cpu}" \
        --max-episodes "${MAX_EP:-5000}" --record-first 100 --record-every 400 \
        --run-dir "$RUN_DIR" --session-id "$(date +%Y%m%d_%H%M%S)" ;;
  record)
    REC_DIR="$RUN_DIR/videos" REC_LOG="$RUN_DIR/train.log" "$PY" "$SCRIPTS/periodic_recorder.py" ;;
  stop)
    pkill -f "balance.world" 2>/dev/null || true
    pkill -f "ppo_balance.py" 2>/dev/null || true
    echo "stopped our sim + trainer" ;;
  *)
    grep '^#' "$0" | sed 's/^# \{0,1\}//' ;;
esac
