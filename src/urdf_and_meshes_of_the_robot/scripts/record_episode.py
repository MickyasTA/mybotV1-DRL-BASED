#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_episode.py — render an MP4 "recording" of a balance episode from the
robot's recorded trajectory (base pose + wheel angles), using forward kinematics
on the actual STL meshes.

Why not gzclient? The Gazebo GUI can't render this robot's STL meshes on WSL
(OpenGL crash), so we reconstruct the motion offline with matplotlib (Agg, fully
headless) — same approach that produced the static robot_render.png.

Stage-1 robot: the legs are rigid, so body+legs move as ONE rigid assembly at
the base pose; only the two wheels additionally spin about their axles.

Public API:
    rec = EpisodeRecorder(mesh_dir)          # loads meshes once (reuse across milestones)
    rec.render(traj, out_path, title=...)    # traj: list of (x,y,z,roll,pitch,yaw,wL,wR)
"""

import os
import struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ----------------------------- math helpers -------------------------------- #
def _rpy(r, p, y):
    cr, sr = np.cos(r), np.sin(r); cp, sp = np.cos(p), np.sin(p); cy, sy = np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _T(xyz, r):
    M = np.eye(4); M[:3, :3] = _rpy(*r); M[:3, 3] = xyz; return M


def _Rz(a):
    M = np.eye(4); c, s = np.cos(a), np.sin(a)
    M[0, 0] = c; M[0, 1] = -s; M[1, 0] = s; M[1, 1] = c
    return M


def _load_stl(path):
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    tris = np.zeros((n, 3, 3), np.float32); off = 84
    for i in range(n):
        tris[i] = np.array(struct.unpack("<9f", d[off + 12:off + 48])).reshape(3, 3)
        off += 50
    return tris


def _cyl(r, h, n=24):
    th = np.linspace(0, 2 * np.pi, n, endpoint=False); t = []
    for i in range(n):
        j = (i + 1) % n
        a = [r * np.cos(th[i]), r * np.sin(th[i]), -h / 2]; b = [r * np.cos(th[j]), r * np.sin(th[j]), -h / 2]
        c = [r * np.cos(th[i]), r * np.sin(th[i]), h / 2];  d = [r * np.cos(th[j]), r * np.sin(th[j]), h / 2]
        t += [[a, b, d], [a, d, c], [[0, 0, -h / 2], b, a], [[0, 0, h / 2], c, d]]
    return np.array(t, np.float32)


def _xf(tris, M):
    return tris @ M[:3, :3].T + M[:3, 3]


# FK link poses in the BODY frame (from the verified static render). Legs rigid.
def _link_poses():
    P = {}
    P["BODY_link"] = np.eye(4)
    hipL = _T([0.065, -0.0025, -0.055], [0, 0, 0])
    P["Upper_left_leg"] = hipL @ _T([0, 0, 0], [1.5708, -0.5686, 1.5708])
    P["Lower_left_leg"] = P["Upper_left_leg"] @ _T([0, -0.24941, 0.032], [0, 0, -1.2888])
    P["Left_wheel"] = P["Lower_left_leg"] @ _T([0, -0.22885, 0.015], [3.1416, 0, 0.81783])
    hipR = _T([-0.065, -0.0025, -0.055], [0, 0, 0])
    P["Upper_right_link"] = hipR @ _T([0, 0, 0], [1.5708, 0.59611, -1.5708])
    P["Lower_right_link"] = P["Upper_right_link"] @ _T([0, -0.24941, 0.034], [0, 0, 1.3094])
    P["Right_wheel"] = P["Lower_right_link"] @ _T([0, -0.22885, 0.015], [3.1416, 0, 0.41886])
    return P


class EpisodeRecorder:
    """Loads meshes once; render() can be called per milestone."""

    LIGHT = np.array([0.4, 0.5, 0.8]) / np.linalg.norm([0.4, 0.5, 0.8])

    def __init__(self, mesh_dir, max_body_tris=2000):
        P = _link_poses()
        body_links = [("BODY_link", (0.85, 0.87, 0.90)),
                      ("Upper_left_leg", (0.55, 0.58, 0.62)), ("Lower_left_leg", (0.55, 0.58, 0.62)),
                      ("Upper_right_link", (0.55, 0.58, 0.62)), ("Lower_right_link", (0.55, 0.58, 0.62))]
        # rigid body+legs: pre-place every triangle in the BODY frame, with a base colour
        tris, cols = [], []
        for name, col in body_links:
            p = os.path.join(mesh_dir, name + ".STL")
            if not os.path.exists(p):
                continue
            w = _xf(_load_stl(p), P[name])
            tris.append(w); cols.append(np.tile(col, (len(w), 1)))
        self.body_tris = np.concatenate(tris) if tris else np.zeros((0, 3, 3))
        self.body_basecol = np.concatenate(cols) if cols else np.zeros((0, 3))
        # DECIMATE: matplotlib 3D depth-sorts every triangle each frame, so a
        # full ~10k-tri body makes rendering ~0.5 s/frame. Evenly subsample to a
        # cap; the robot stays recognizable and renders ~3-4x faster.
        if len(self.body_tris) > max_body_tris:
            sel = np.linspace(0, len(self.body_tris) - 1, max_body_tris).astype(int)
            self.body_tris = self.body_tris[sel]
            self.body_basecol = self.body_basecol[sel]
        # wheels: cylinder in wheel-local frame + their body-frame placement
        self.wheel_tris = _cyl(0.0826, 0.0422)
        self.P_wheelL = P["Left_wheel"]
        self.P_wheelR = P["Right_wheel"]
        self.wheel_col = np.array((0.12, 0.12, 0.13))

    def _shade(self, world_tris, basecol):
        nrm = np.cross(world_tris[:, 1] - world_tris[:, 0], world_tris[:, 2] - world_tris[:, 0])
        ln = np.linalg.norm(nrm, axis=1, keepdims=True); ln[ln == 0] = 1; nrm = nrm / ln
        sh = 0.45 + 0.55 * np.clip(np.abs(nrm @ self.LIGHT), 0, 1)
        if basecol.ndim == 1:
            basecol = basecol[None, :]
        return np.clip(basecol * sh[:, None], 0, 1)

    def render(self, traj, out_path, title="mybot balance", fps=25, max_frames=160):
        """traj: list/array of (x,y,z,roll,pitch,yaw,wheelL_angle,wheelR_angle)."""
        traj = np.asarray(traj, dtype=np.float32)
        if len(traj) == 0:
            return None
        # subsample to at most max_frames so the render stays quick
        step = max(1, len(traj) // max_frames)
        idx = np.arange(0, len(traj), step)
        frames = traj[idx]

        fig = plt.figure(figsize=(7, 8))
        ax = fig.add_subplot(111, projection="3d")

        # world extent: follow the robot, fixed radius so tilt/fall is visible
        cx, cy = float(np.median(traj[:, 0])), float(np.median(traj[:, 1]))
        R = 0.6

        def draw(i):
            ax.clear()
            x, y, z, roll, pitch, yaw, wl, wr = frames[i]
            Tw = _T([x, y, z], [roll, pitch, yaw])
            # ground grid
            gx = np.linspace(cx - R, cx + R, 7); gy = np.linspace(cy - R, cy + R, 7)
            for xx in gx:
                ax.plot([xx, xx], [cy - R, cy + R], [0, 0], color="0.85", lw=0.6)
            for yy in gy:
                ax.plot([cx - R, cx + R], [yy, yy], [0, 0], color="0.85", lw=0.6)
            # body+legs (rigid)
            bw = _xf(self.body_tris, Tw)
            pc = Poly3DCollection(bw, linewidths=0); pc.set_facecolor(self._shade(bw, self.body_basecol))
            ax.add_collection3d(pc)
            # wheels (spin about local axle)
            for Pw, ang in ((self.P_wheelL, wl), (self.P_wheelR, wr)):
                ww = _xf(self.wheel_tris, Tw @ Pw @ _Rz(ang))
                pcw = Poly3DCollection(ww, linewidths=0); pcw.set_facecolor(self._shade(ww, self.wheel_col))
                ax.add_collection3d(pcw)
            ax.set_xlim(cx - R, cx + R); ax.set_ylim(cy - R, cy + R); ax.set_zlim(0, 2 * R)
            ax.set_box_aspect((1, 1, 1))
            tilt = float(np.degrees(np.hypot(roll, pitch)))
            ax.set_title(f"{title}\nframe {i+1}/{len(frames)}  t={idx[i]*0.02:5.2f}s  tilt={tilt:4.1f}deg  z={z:.2f}")
            ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
            ax.view_init(elev=14, azim=-72)

        # prefer MP4 (ffmpeg); fall back to GIF (Pillow, needs no external binary)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        attempts = [(FFMpegWriter(fps=fps, bitrate=2400), out_path),
                    (PillowWriter(fps=fps), os.path.splitext(out_path)[0] + ".gif")]
        last_err = None
        for writer, final in attempts:
            try:
                with writer.saving(fig, final, dpi=90):
                    for i in range(len(frames)):
                        draw(i)
                        writer.grab_frame()
                plt.close(fig)
                return final
            except Exception as e:
                last_err = e
        plt.close(fig)
        print(f"[record] render failed (mp4 and gif): {last_err}")
        return None


# Standalone smoke test: synthesize a wobble and render it.
if __name__ == "__main__":
    import sys
    mesh_dir = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.expanduser("~/ros2_ws/src/urdf_and_meshes_of_the_robot/meshes")
    out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/test_rollout.mp4"
    n = 200
    t = np.linspace(0, 4 * np.pi, n)
    traj = np.stack([0.01 * t, 0 * t, 0.53 + 0 * t, 0.15 * np.sin(t), 0.0 * t, 0 * t, t, t], axis=1)
    rec = EpisodeRecorder(mesh_dir)
    print("wrote", rec.render(traj, out, title="smoke test"))
