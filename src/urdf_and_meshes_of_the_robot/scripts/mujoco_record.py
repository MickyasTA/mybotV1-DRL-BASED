#!/usr/bin/env python3
"""
Backward-compatible shim -> mujoco_view.py (the single auto-detecting viewer/recorder
that works for EVERY stage: balance, goal-nav, and later). Kept so old commands keep
working; new code should just use mujoco_view.py --record.

  MUJOCO_GL=osmesa python mujoco_record.py --ckpt <ckpt> --out clip.mp4
"""
import os
import sys
import argparse

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

if __name__ == "__main__":
    from mujoco_view import record
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--max-steps", type=int, default=1000)
    p.add_argument("--fps", type=int, default=50)
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()
    record(a.ckpt, a.out, a.max_steps, a.fps, a.seed)
