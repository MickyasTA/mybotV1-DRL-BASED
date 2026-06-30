#!/usr/bin/env python3
"""
camera_recorder.py — record REAL Gazebo footage of an eval rollout by capturing
the headless camera sensor (/eval_cam/...) defined in balance.world.

gzclient (the Gazebo GUI) crashes on this robot's meshes under WSL, but a Gazebo
*camera sensor* renders fine when gzserver runs under Xvfb + software GL
(LIBGL_ALWAYS_SOFTWARE=1). This subscribes to that camera's image topic and pipes
frames to ffmpeg -> MP4. Has its OWN node/executor so it can be pumped between env
steps during the synchronous-spin eval.

Usage:
    rec = CameraRecorder(topic="/eval_cam/eval_cam/image_raw")
    rec.start(out_path, fps=30)
    ... for each eval step: env.step(a); rec.pump() ...
    rec.finish()         # -> out_path (or None if no frames / ffmpeg missing)
"""

import subprocess
import shutil
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image


class CameraRecorder:
    def __init__(self, topic="/eval_cam/eval_cam/image_raw"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("camera_recorder")
        self.exec = SingleThreadedExecutor()
        self.exec.add_node(self.node)
        self.node.create_subscription(Image, topic, self._cb, 10)
        self._latest = None
        self._w = self._h = None
        self._ff = None
        self._out = None
        self._frames = 0

    def _cb(self, msg: Image):
        # gazebo_ros camera publishes rgb8
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        self._latest = (msg.width, msg.height, arr)

    def available(self):
        """Wait briefly for the camera to produce a first frame (it renders only
        once subscribed). Returns True if frames are flowing."""
        # short wait: on WSL the Gazebo camera can't render this robot's STL meshes
        # (OGRE limit), so there is normally no camera topic -> fall back to the FK
        # reconstruction quickly. (Kept for native-Linux runs where the camera works.)
        import time as _t
        t0 = _t.time()
        while self._latest is None and _t.time() - t0 < 1.5:
            self.exec.spin_once(timeout_sec=0.1)
        return self._latest is not None

    def start(self, out_path, fps=30):
        if shutil.which("ffmpeg") is None:
            self.node.get_logger().warn("ffmpeg not found; camera recording disabled")
            return False
        if not self.available():
            self.node.get_logger().warn("no camera frames; recording disabled")
            return False
        w, h, _ = self._latest
        self._w, self._h, self._out = w, h, out_path
        self._ff = subprocess.Popen(
            ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
             "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", out_path],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._frames = 0
        return True

    def pump(self, max_spins=6):
        """Process any pending camera frames and write the latest to the video."""
        if self._ff is None:
            return
        for _ in range(max_spins):
            self.exec.spin_once(timeout_sec=0.0)
        if self._latest is None:
            return
        w, h, arr = self._latest
        if w == self._w and h == self._h and arr.size == w * h * 3:
            try:
                self._ff.stdin.write(arr.tobytes())
                self._frames += 1
            except (BrokenPipeError, ValueError):
                pass

    def finish(self):
        if self._ff is None:
            return None
        try:
            self._ff.stdin.close()
            self._ff.wait(timeout=30)
        except Exception:
            self._ff.kill()
        out = self._out if self._frames > 0 else None
        self._ff = None
        self._frames = 0
        return out

    def close(self):
        try:
            self.exec.shutdown()
            self.node.destroy_node()
        except Exception:
            pass
