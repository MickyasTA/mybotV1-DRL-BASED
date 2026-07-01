#!/usr/bin/env python3
"""
Backward-compatible shim -> mujoco_view.py (the single auto-detecting viewer/recorder
that works for EVERY stage: balance, goal-nav, and later). Kept so old commands keep
working; new code should just use mujoco_view.py.

  DISPLAY=:0 python mujoco_play.py --ckpt <run>/model_best.pth      # live viewer
  # headless mp4:  MUJOCO_GL=osmesa python mujoco_view.py --ckpt <ckpt> --record clip.mp4
"""
import os
import sys
import argparse

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

if __name__ == "__main__":
    from mujoco_view import play
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--loops", type=int, default=1000)
    p.add_argument("--seed", type=int, default=None)
    a = p.parse_args()
    play(a.ckpt, a.loops, a.seed)
