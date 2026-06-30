#!/usr/bin/env python3
"""
periodic_recorder.py — periodic REAL-Gazebo video recording during HEADLESS training.

Ported from collision_avoidance_system/scripts/periodic_recorder.py. Every REC_EVERY
completed episodes it SPAWNS a temporary camera entity into the running (headless)
Gazebo sim, records REC_EPISODES episodes of its image stream to an mp4, then DELETES
the camera — so the sim stays headless except during the brief recording windows, and
you get REAL Gazebo footage (gzserver renders the camera) without ever running gzclient
(which can't render this robot's STL meshes on WSL).

Standalone ROS 2 node, run alongside the trainer; reads the episode number from the
training log file, so it needs zero coupling to the trainer.

Env knobs:
  REC_EVERY=400        record at every Nth episode
  REC_EPISODES=40      how many episodes to record each time
  REC_MAX_SECONDS=120  hard cap on a single clip's wall-clock length
  REC_FPS=20           output video fps
  REC_LOG=<path>       training log to read episode numbers from
  REC_DIR=<path>       output dir for the mp4s
  REC_CAM_SDF=<path>   camera model sdf to spawn (default /tmp/rec_cam.sdf)
"""
import os
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

REPO = "/mnt/c/Users/mtasfaw/Documents/personal/mybotV1-DRL-BASED"
RUN = f"{REPO}/training_results/run1"
REC_EVERY = int(os.environ.get("REC_EVERY", "400"))
REC_EPISODES = int(os.environ.get("REC_EPISODES", "40"))
REC_MAX_SECONDS = float(os.environ.get("REC_MAX_SECONDS", "120"))
REC_FPS = float(os.environ.get("REC_FPS", "20"))
REC_LOG = os.environ.get("REC_LOG", f"{RUN}/train.log")
REC_DIR = os.environ.get("REC_DIR", f"{RUN}/videos")
REC_CAM_SDF = os.environ.get("REC_CAM_SDF", "/tmp/rec_cam_mybot.sdf")
IMG_TOPIC = "/rec_cam/rec_camera/image_raw"
# mybot logs episodes as "... eps <N>/<max> ..." on each PPO update line.
_EP_RE = re.compile(r"eps (\d+)/")

# Temporary recording camera. The balance robot is small (~0.5 m, near the origin),
# so this is a close 3/4 side view (vs the drone's 30 m overhead): it sits at
# (1.6,-1.6,1.1) looking down ~0.3 rad toward the robot, so the body tilt + spinning
# wheels are clearly visible. gzserver renders it headless; gazebo_ros_camera
# publishes /rec_cam/rec_camera/image_raw.
_CAM_SDF_XML = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="rec_cam">
    <static>true</static>
    <pose>1.6 -1.6 1.1 0 0.3 2.356</pose>
    <link name="rec_cam_link">
      <visual name="v"><geometry><box><size>0.05 0.05 0.05</size></box></geometry></visual>
      <sensor name="rec_camera" type="camera">
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>1.1</horizontal_fov>
          <image><width>960</width><height>540</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <plugin name="rec_cam_plugin" filename="libgazebo_ros_camera.so">
          <ros><namespace>/rec_cam</namespace></ros>
          <camera_name>rec_camera</camera_name>
          <frame_name>rec_cam_link</frame_name>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
"""


class PeriodicRecorder(Node):
    def __init__(self):
        super().__init__("periodic_recorder")
        os.makedirs(REC_DIR, exist_ok=True)
        try:
            with open(REC_CAM_SDF, "w") as f:
                f.write(_CAM_SDF_XML)
        except Exception as e:
            self.get_logger().warn(f"could not write camera sdf {REC_CAM_SDF}: {e}")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.writer = None
        self.video_path = None
        self.recording = False
        self.rec_start_ep = 0
        self.rec_start_t = 0.0
        self.frames = 0
        self.last_mark = 0
        self.create_subscription(Image, IMG_TOPIC, self.on_image, qos_profile_sensor_data)
        self.create_timer(3.0, self.tick)
        self.get_logger().info(
            f"periodic_recorder: record {REC_EPISODES} eps every {REC_EVERY} eps "
            f"(<= {REC_MAX_SECONDS:.0f}s) -> {REC_DIR}")

    def current_episode(self) -> int:
        ep = 0
        try:
            with open(REC_LOG, "rb") as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 65536))
                tail = f.read().decode("utf-8", errors="ignore")
            for m in _EP_RE.finditer(tail):
                ep = int(m.group(1))
        except Exception:
            pass
        return ep

    def tick(self):
        ep = self.current_episode()
        if not self.recording:
            mark = (ep // REC_EVERY) * REC_EVERY
            if mark >= REC_EVERY and mark > self.last_mark:
                self._start(ep, mark)
        else:
            if (ep - self.rec_start_ep) >= REC_EPISODES or \
                    (time.time() - self.rec_start_t) >= REC_MAX_SECONDS:
                self._stop(ep)

    def _spawn_cam(self):
        subprocess.run(
            ["ros2", "run", "gazebo_ros", "spawn_entity.py", "-entity", "rec_cam",
             "-file", REC_CAM_SDF, "-x", "1.6", "-y", "-1.6", "-z", "1.1",
             "-R", "0", "-P", "0.3", "-Y", "2.356"],
            capture_output=True, timeout=25)

    def _delete_cam(self):
        subprocess.run(
            ["ros2", "service", "call", "/delete_entity",
             "gazebo_msgs/srv/DeleteEntity", "{name: 'rec_cam'}"],
            capture_output=True, timeout=15)

    def _start(self, ep, mark):
        self.get_logger().info(f"[REC] ep {ep} reached mark {mark}: spawning camera, recording...")
        try:
            self._spawn_cam()
        except Exception as e:
            self.get_logger().warn(f"[REC] camera spawn failed: {e}")
        time.sleep(3.0)
        with self.lock:
            self.writer = None
            self.video_path = os.path.join(REC_DIR, f"gazebo_ep{mark}.mp4")
            self.frames = 0
            self.rec_start_ep = ep
            self.rec_start_t = time.time()
            self.recording = True
            self.last_mark = mark

    def on_image(self, msg: Image):
        with self.lock:
            if not self.recording:
                return
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                return
            if self.writer is None:
                h, w = img.shape[:2]
                self.writer = cv2.VideoWriter(
                    self.video_path, cv2.VideoWriter_fourcc(*"mp4v"), REC_FPS, (w, h))
            self.writer.write(img)
            self.frames += 1

    def _stop(self, ep):
        with self.lock:
            path, frames = self.video_path, self.frames
            if self.writer is not None:
                self.writer.release()
                self.writer = None
            self.recording = False
        try:
            self._delete_cam()
        except Exception as e:
            self.get_logger().warn(f"[REC] camera delete failed: {e}")
        self.get_logger().info(
            f"[REC] saved {path} ({frames} frames, eps {self.rec_start_ep}..{ep}); back to headless")


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.recording:
                node._delete_cam()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
