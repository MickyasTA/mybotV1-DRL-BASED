#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
balance_env.py — Gymnasium environment for the "self-balance in place" milestone.

Bridges a from-scratch PPO (see ppo_balance.py) to the two-wheeled bipedal robot
running in Gazebo Classic via ros2_control.

Robot DOFs used here:
  * 2 wheels  (Left_joint, Right_joint)              -> EFFORT (torque) control  [RL action]
  * 6 hips    (yaw,roll,pitch) x (left,right)         -> POSITION control          [RL action]
  * 2 knees   (Lower_left_joint, Lower_right_joint)   -> POSITION, held at q_nom

Action (8-D, symmetric [-1,1]):
  [ wheelL, wheelR,  hip_yaw_L, hip_roll_L, hip_pitch_L,  hip_yaw_R, hip_roll_R, hip_pitch_R ]
  wheels   -> tau   = a * max_wheel_effort
  hips     -> q*    = q_hip_nom + a * hip_action_scale     (PD-tracked by ros2_control)

Observation (proprioceptive, raw units; normalization happens in the PPO):
  per frame = [ roll, pitch, wx, wy, wz, wheelL_vel, wheelR_vel, base_vx, base_vy,
                hip_pos(6), hip_vel(6), prev_action(8) ]   (29)
  stacked over `history_len` frames to handle latency / non-Markovianity.

Deterministic reset: pause -> set_entity_state (upright, ZERO velocity) -> command
nominal stance -> settle. Stepping is /clock-paced (works with real_time_update_rate=0).

"""

import math
import threading
import time
from collections import deque

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState


# ----------------------------- configuration ------------------------------- #
ROBOT_NAME = "urdf_and_meshes_of_the_robot"

WHEEL_JOINTS = ["Left_joint", "Right_joint"]
# Order MUST match config/controllers.yaml hip_position_controller.joints
HIP_JOINTS = ["hip_yaw_left", "hip_roll_left", "Upper_left_joint",
              "hip_yaw_right", "hip_roll_right", "Upper_right_joint"]
KNEE_JOINTS = ["Lower_left_joint", "Lower_right_joint"]
# Effort command order — MUST match config/controllers.yaml joint_effort_controller.joints
JOINT_ORDER = WHEEL_JOINTS + HIP_JOINTS + KNEE_JOINTS   # 10 joints
# Legs handled by the JointTrajectoryController — order MUST match leg_traj_controller.joints
LEG_JOINTS = HIP_JOINTS + KNEE_JOINTS                   # 8 joints
# Per-joint PD gains for the EFFORT hold of the 8 leg joints (stage-1 rigid stance).
# Effort is the only sim path that develops real holding torque for these load-bearing
# chained joints (position/position_pid interfaces teleport/fail to hold in Gazebo Classic).
LEG_KP = 400.0
LEG_KD = 20.0
LEG_EFFORT_MAX = 200.0   # clamp; must be <= the leg joint <limit effort> in the URDF

SRV_PAUSE = "/pause_physics"
SRV_UNPAUSE = "/unpause_physics"
SRV_RESET_WORLD = "/reset_world"
SRV_SET_STATE = "/gazebo/set_entity_state"
TOPIC_MODEL_STATES = "/gazebo/model_states"


def quat_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    """Hamilton product a ⊗ b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# The IMU_link is mounted rpy=(pi/2,0,0) on the body. We read tilt/rate from the
# IMU (a real sensor that NEVER wedges) instead of /gazebo/model_states
# (gazebo_ros_state intermittently wedges to all-zeros on this WSL setup, which
# corrupted the training signal). Body orientation = q_imu ⊗ inv(Rx(pi/2)),
# validated against model_states (matches to <0.01 rad once settled).
_INV_IMU_MOUNT = euler_to_quat(-math.pi / 2.0, 0.0, 0.0)
# Base forward velocity from the wheel encoders (avoids model_states twist too).
WHEEL_RADIUS = 0.0826   # m (matches the wheel collision cylinder)


class BalanceEnv(gym.Env):
    """Two-wheeled bipedal self-balancing env with 3-DOF actuated hips."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        control_dt: float = 0.02,        # 50 Hz policy rate (sim seconds per step)
        max_steps: int = 1000,           # 20 s episodes
        history_len: int = 3,
        max_wheel_effort: float = 1.5,   # Nm for |wheel action|=1 (light wheels: 6 Nm launches it!)
        max_joint_effort: float = 4.0,   # Nm clamp on every commanded torque (light links)
        hip_action_scale: float = 0.1,   # rad for |hip action|=1, tiny deviations about q_hip_nom
        kp_hip: float = 20.0, kd_hip: float = 1.0,     # hip position->torque PD gains (hold, gentle)
        kp_knee: float = 25.0, kd_knee: float = 1.2,   # knee position->torque PD gains
        fall_tilt: float = 0.5,          # rad (~29 deg) -> fallen
        spawn_z: float = 0.53,           # reset height; FK: wheel contact ~0.52 m below base
        reset_tilt_noise: float = 0.03,  # rad random initial tilt
        q_hip_nom=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # nominal hip stance (TUNE)
        q_knee_nom=(0.0, 0.0),                       # nominal knee stance (TUNE)
        use_pause: bool = False,   # False = continuous sim (no pause/unpause) -> much faster (10-20x)
        # reward weights (tune empirically — paper values were refuted)
        w_alive: float = 1.0,
        w_tilt: float = 2.0,        # gentler tilt penalty (early tilts are unavoidable)
        w_tilt_rate: float = 0.0,   # 0: don't punish the fast CORRECTIVE motion needed to catch a fall
        w_effort: float = 0.005,
        w_drift: float = 0.05,      # light: balancing REQUIRES driving the wheels; 0.5 punished the solution
        fall_penalty: float = 10.0,
        sensor_timeout: float = 30.0,
    ):
        super().__init__()
        self.control_dt = float(control_dt)
        self.max_steps = int(max_steps)
        self.history_len = int(history_len)
        self.max_wheel_effort = float(max_wheel_effort)
        self.max_joint_effort = float(max_joint_effort)
        self.hip_action_scale = float(hip_action_scale)
        self.kp_hip = float(kp_hip); self.kd_hip = float(kd_hip)
        self.kp_knee = float(kp_knee); self.kd_knee = float(kd_knee)
        self.fall_tilt = float(fall_tilt)
        self.spawn_z = float(spawn_z)
        self.reset_tilt_noise = float(reset_tilt_noise)
        self.q_hip_nom = np.array(q_hip_nom, dtype=np.float32)
        self.q_knee_nom = np.array(q_knee_nom, dtype=np.float32)
        self.use_pause = bool(use_pause)
        self.w_alive = w_alive
        self.w_tilt = w_tilt
        self.w_tilt_rate = w_tilt_rate
        self.w_effort = w_effort
        self.w_drift = w_drift
        self.fall_penalty = fall_penalty
        self.sensor_timeout = sensor_timeout

        self.n_wheel = len(WHEEL_JOINTS)   # 2
        self.n_hip = len(HIP_JOINTS)       # 6
        self.act_dim = self.n_wheel   # 2 (STAGE 1: wheels only; legs are rigid fixed joints)

        # --- spaces ---
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.act_dim,), dtype=np.float32)
        # frame = 9 base/imu/wheel + hip_pos(6) + hip_vel(6) + prev_action(act_dim)
        self._frame_dim = 9 + 2 * self.n_hip + self.act_dim
        obs_dim = self._frame_dim * self.history_len
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)

        # --- ROS state ---
        # Tilt/pose come from /gazebo/model_states (the IMU sensor is broken on
        # this sim: orientation is frozen and accel/gyro are NaN). model_states
        # reads are stable; we just avoid the gazebo_ros_state WRITE op
        # (set_entity_state), which is what wedges it -> reset via reset_world only.
        self._lock = threading.Lock()
        self._joint_vel = {}
        self._joint_pos = {}
        self._base_vel = (0.0, 0.0)
        self._base_pos = (0.0, 0.0, self.spawn_z)
        self._base_rpy = (0.0, 0.0, 0.0)   # body tilt from model_states (true orientation)
        self._base_ang = (0.0, 0.0, 0.0)   # body angular velocity
        self._got_model = False
        self._imu_rpy = (0.0, 0.0, 0.0)    # body tilt from IMU (primary tilt source)
        self._imu_ang = (0.0, 0.0, 0.0)    # body angular velocity from IMU gyro
        self._got_imu = False
        self._sim_time = 0.0
        self._last_action = np.zeros(self.act_dim, dtype=np.float32)
        self._hist = deque(maxlen=self.history_len)
        self._steps = 0
        self._wedge_strikes = 0

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("balance_env")  # NOTE: deliberately NOT use_sim_time

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)

        self.wheel_pub = self.node.create_publisher(
            Float64MultiArray, "/wheel_effort_controller/commands", 10)
        # The 8 leg joints are FREE (revolute) and SEPARATELY CONTROLLED: the env holds
        # each at the zero stance with a per-joint PD -> EFFORT command (JointGroupEffort
        # Controller). Effort is the only sim path that develops real holding torque for
        # these load-bearing chained joints. Stage 2+ can replace the PD hold with
        # RL-commanded leg efforts to walk / climb.
        self.leg_pub = self.node.create_publisher(
            Float64MultiArray, "/leg_effort_controller/commands", 10)
        self.n_leg = len(LEG_JOINTS)   # 8 (3-DOF hip + knee per leg)

        self.node.create_subscription(JointState, "/joint_states", self._joint_cb, sensor_qos)
        self.node.create_subscription(ModelStates, TOPIC_MODEL_STATES, self._model_cb, 10)
        # IMU is the PRIMARY tilt/rate source: it's a real sensor that reports valid
        # orientation (verified: no NaN, tracks true pose with the known 90° mount
        # offset) and — unlike the gazebo_ros_state model_states plugin — does NOT
        # intermittently wedge to zeros. Reading tilt from model_states corrupted the
        # training signal (fake-long episodes when it under-reported tilt). model_states
        # is kept only for base x/y velocity (drift term).
        self.node.create_subscription(Imu, "/imu", self._imu_cb, sensor_qos)
        self.node.create_subscription(Clock, "/clock", self._clock_cb, sensor_qos)

        self.cli_pause = self.node.create_client(Empty, SRV_PAUSE)
        self.cli_unpause = self.node.create_client(Empty, SRV_UNPAUSE)
        self.cli_reset = self.node.create_client(Empty, SRV_RESET_WORLD)
        self.cli_set_state = self.node.create_client(SetEntityState, SRV_SET_STATE)

        # SYNCHRONOUS spinning — NO background spin thread. A background thread
        # spinning the executor floods the GIL processing /clock + sensor topics
        # at high real-time factor, which starves the main thread's per-step
        # policy inference and collapsed training throughput (~19 sps although the
        # env itself does ~700 sps). Spinning manually inside step()/reset()
        # serializes ROS work with policy work on one thread: no GIL contention,
        # and sensor state is fresh exactly when we read it.
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)

        self._wait_for_services()
        self._wait_for_sensors()
        self.node.get_logger().info(
            f"BalanceEnv ready. action_dim={self.act_dim} obs_dim={obs_dim}")

    # ----------------------------- ROS callbacks --------------------------- #
    def _imu_cb(self, msg: Imu):
        o = msg.orientation
        # body orientation = q_imu ⊗ inv(q_mount)  (undo the 90° IMU mount)
        bx, by, bz, bw = quat_mul((o.x, o.y, o.z, o.w), _INV_IMU_MOUNT)
        roll, pitch, yaw = quat_to_euler(bx, by, bz, bw)
        # angular velocity: rotate the IMU-frame gyro into the base frame by the
        # mount rotation R(pi/2, X): (gx, gy, gz) -> (gx, -gz, gy)
        gx, gy, gz = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if any(math.isnan(v) for v in (bx, by, bz, bw, gx, gy, gz)):
            return                               # ignore a bad IMU frame
        with self._lock:
            self._imu_rpy = (roll, pitch, yaw)
            self._imu_ang = (gx, -gz, gy)
            self._got_imu = True

    def _model_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(ROBOT_NAME)
        except ValueError:
            return
        p = msg.pose[idx].position
        o = msg.pose[idx].orientation
        tl = msg.twist[idx].linear
        ta = msg.twist[idx].angular
        roll, pitch, yaw = quat_to_euler(o.x, o.y, o.z, o.w)
        with self._lock:
            self._base_pos = (p.x, p.y, p.z)
            self._base_vel = (tl.x, tl.y)
            self._base_rpy = (roll, pitch, yaw)
            self._base_ang = (ta.x, ta.y, ta.z)
            self._got_model = True

    def _joint_cb(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._joint_pos[name] = msg.position[i]
                if i < len(msg.velocity):
                    self._joint_vel[name] = msg.velocity[i]

    def _clock_cb(self, msg: Clock):
        with self._lock:
            self._sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ----------------------------- helpers --------------------------------- #
    def _wait_for_services(self):
        for cli, name in [(self.cli_pause, SRV_PAUSE), (self.cli_unpause, SRV_UNPAUSE),
                          (self.cli_reset, SRV_RESET_WORLD), (self.cli_set_state, SRV_SET_STATE)]:
            t0 = time.time()
            while not cli.wait_for_service(timeout_sec=1.0):
                if time.time() - t0 > self.sensor_timeout:
                    self.node.get_logger().warn(f"Service {name} not available; continuing.")
                    break

    def _wait_for_sensors(self):
        t0 = time.time()
        while time.time() - t0 < self.sensor_timeout:
            self._exec.spin_once(timeout_sec=0.05)   # process incoming sensor msgs
            with self._lock:
                ready = (self._got_imu and self._got_model
                         and all(j in self._joint_vel for j in WHEEL_JOINTS))
            if ready:
                return
        self.node.get_logger().warn("Timed out waiting for imu/model_states/joint sensors; obs may be stale.")

    def _call(self, client, request, timeout=5.0):
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
        t0 = time.time()
        while not future.done() and time.time() - t0 < timeout:
            self._exec.spin_once(timeout_sec=0.01)   # drive the service response
        return future.result() if future.done() else None

    def _now_sim(self):
        with self._lock:
            return self._sim_time

    def _advance_sim(self, duration):
        if self.use_pause:
            self._call(self.cli_unpause, Empty.Request())
        start = self._now_sim()
        wall0 = time.time()
        # Spin the node ourselves: each spin_once processes a pending /clock (and
        # sensor) message, advancing _sim_time, until `duration` of sim has passed.
        # No background thread -> no GIL fight with the policy forward pass.
        while self._now_sim() - start < duration:
            self._exec.spin_once(timeout_sec=0.002)
            if time.time() - wall0 > max(2.0, duration * 50):
                break
        if self.use_pause:
            self._call(self.cli_pause, Empty.Request())

    # ----- commands: wheels = policy torque (effort); legs = position targets sent to
    # the 1 kHz JointTrajectoryController (smart high-rate PD; sim-to-real friendly).
    # A 50 Hz PD inside this env was too slow for the light legs (they flung); the JTC
    # runs the PID->torque loop at the controller_manager rate (1 kHz) instead. -----
    def _joint_pv(self, names):
        with self._lock:
            pos = np.array([self._joint_pos.get(j, 0.0) for j in names], dtype=np.float32)
            vel = np.array([self._joint_vel.get(j, 0.0) for j in names], dtype=np.float32)
        pos = np.nan_to_num(pos, nan=0.0, posinf=0.0, neginf=0.0)
        vel = np.nan_to_num(vel, nan=0.0, posinf=0.0, neginf=0.0)
        return pos, vel

    def _command_wheels(self, a_wheel):
        tau = np.nan_to_num(np.clip(a_wheel, -1.0, 1.0) * self.max_wheel_effort)
        self.wheel_pub.publish(Float64MultiArray(data=[float(x) for x in tau]))

    def _hold_legs(self):
        # Stage 1: hold each leg joint at the zero stance with a per-joint PD that
        # outputs an EFFORT (torque) -> a real physics hold (SetForce), so the legs
        # stay rigid and the robot is a proper inverted pendulum on its wheels.
        # Command order MUST match config/controllers.yaml leg_effort_controller.joints
        # (== LEG_JOINTS).
        pos, vel = self._joint_pv(LEG_JOINTS)
        tau = np.clip(-LEG_KP * pos - LEG_KD * vel, -LEG_EFFORT_MAX, LEG_EFFORT_MAX)
        self.leg_pub.publish(Float64MultiArray(data=[float(x) for x in tau]))

    def _command_zero(self):
        self._command_wheels(np.zeros(self.n_wheel, dtype=np.float32))

    def _apply_action(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        self._command_wheels(action[:self.n_wheel])   # RL controls the 2 wheel torques
        self._hold_legs()                              # legs actively held at stance (stage 1)

    def _read_frame(self):
        # ALL signals come from the IMU + wheel encoders now — NOT model_states.
        # Tilt + angular rate from the IMU; base forward velocity from wheel
        # odometry (mean wheel speed × radius). This makes the obs/reward fully
        # independent of the flaky gazebo_ros_state plugin.
        with self._lock:
            roll, pitch, _yaw = self._imu_rpy
            wx, wy, wz = self._imu_ang
            wl = self._joint_vel.get("Left_joint", 0.0)
            wr = self._joint_vel.get("Right_joint", 0.0)
            hip_pos = [self._joint_pos.get(j, 0.0) for j in HIP_JOINTS]
            hip_vel = [self._joint_vel.get(j, 0.0) for j in HIP_JOINTS]
        bvx = 0.5 * (wl + wr) * WHEEL_RADIUS   # wheel-odometry forward velocity
        bvy = 0.0
        frame = np.array(
            [roll, pitch, wx, wy, wz, wl, wr, bvx, bvy]
            + hip_pos + hip_vel
            + list(self._last_action),
            dtype=np.float32)
        # sanitize: never emit NaN/inf (unstable dynamics can blow up velocities)
        frame = np.nan_to_num(frame, nan=0.0, posinf=1e3, neginf=-1e3)
        return np.clip(frame, -1e3, 1e3)

    def _get_obs(self):
        return np.concatenate(list(self._hist), dtype=np.float32)

    def _fallen(self, frame):
        return abs(frame[0]) > self.fall_tilt or abs(frame[1]) > self.fall_tilt

    def _reward(self, frame, action):
        roll, pitch = frame[0], frame[1]
        wx, wy = frame[2], frame[3]
        bvx, bvy = frame[7], frame[8]
        tilt = roll * roll + pitch * pitch
        tilt_rate = wx * wx + wy * wy
        effort = float(np.sum(np.square(action)))   # over all 8 action dims
        drift = bvx * bvx + bvy * bvy
        return (self.w_alive
                - self.w_tilt * tilt
                - self.w_tilt_rate * tilt_rate
                - self.w_effort * effort
                - self.w_drift * drift)

    # ----------------------------- Gym API --------------------------------- #
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._steps = 0
        self._odom_x = 0.0
        self._last_action = np.zeros(self.act_dim, dtype=np.float32)

        if self.use_pause:
            self._call(self.cli_pause, Empty.Request())

        # reset_world (init plugin, NOT gazebo_ros_state) resets joints + poses +
        # velocities to the spawn state: upright at spawn_z, zero velocity. We use
        # it ALONE -- no set_entity_state WRITE (that's what wedges model_states),
        # no velocity kick (that breaks learning). The robot starts upright+still;
        # tiny numerical asymmetry + the policy's own actions provide the variation.
        self._call(self.cli_reset, Empty.Request())
        if self.use_pause:
            self._call(self.cli_unpause, Empty.Request())

        # advance a few control steps (zero torque) so the IMU CATCHES UP to the
        # upright reset — it lags ~1 step after reset_world, so reading immediately
        # gives the stale pre-reset (fallen) orientation and would false-trigger the
        # wedge guard. The robot tips slowly at first (<0.06 rad in 3 steps), so this
        # still starts the episode essentially upright.
        for _ in range(3):
            self._apply_action(np.zeros(self.act_dim, dtype=np.float32))
            self._advance_sim(self.control_dt)

        frame = self._read_frame()

        # WEDGE GUARD: after reset_world the robot is UPRIGHT, so the IMU body tilt
        # must be ~0. If the IMU reports a large tilt at reset for many CONSECUTIVE
        # resets, the IMU/sim has wedged (frozen) -> raise so the resilient wrapper
        # restarts + resumes instead of training on garbage.
        if abs(frame[0]) > 0.4 or abs(frame[1]) > 0.4:
            self._wedge_strikes += 1
            if self._wedge_strikes >= 10:
                raise RuntimeError(
                    "BalanceEnv: IMU reports fallen at upright reset for 10 "
                    "consecutive resets -> sim/IMU WEDGED. Restart sim + resume.")
        else:
            self._wedge_strikes = 0

        self._hist.clear()
        for _ in range(self.history_len):
            self._hist.append(frame)
        return self._get_obs(), {"sim_time": self._now_sim()}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(self.act_dim)
        self._steps += 1

        self._apply_action(action)
        self._advance_sim(self.control_dt)
        self._last_action = np.clip(action, -1.0, 1.0)

        frame = self._read_frame()
        self._hist.append(frame)
        obs = self._get_obs()

        terminated = self._fallen(frame)
        truncated = self._steps >= self.max_steps
        reward = self._reward(frame, self._last_action)
        if terminated:
            reward -= self.fall_penalty

        info = {
            "sim_time": self._now_sim(), "steps": self._steps,
            "roll": float(frame[0]), "pitch": float(frame[1]),
            "base_pos": self._base_pos, "is_fallen": bool(terminated),
        }
        return obs, float(reward), bool(terminated), bool(truncated), info

    def close(self):
        try:
            self._command_zero()
            if self.use_pause:
                self._call(self.cli_unpause, Empty.Request())
        finally:
            self._exec.shutdown()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


# Quick env-contract smoke test:  python balance_env.py
if __name__ == "__main__":
    env = BalanceEnv()
    obs, info = env.reset()
    print(f"obs_dim={obs.shape}, action_dim={env.action_space.shape}")
    print(f"reset obs finite={np.all(np.isfinite(obs))} head={obs[:6]}")
    ep_r = 0.0
    for i in range(200):
        a = env.action_space.sample()
        obs, r, term, trunc, info = env.step(a)
        ep_r += r
        if i % 25 == 0:
            print(f"step {i:3d}  r={r:+.3f}  roll={info['roll']:+.3f} "
                  f"pitch={info['pitch']:+.3f}  fallen={info['is_fallen']}")
        if term or trunc:
            print(f"episode end @ step {i} (term={term}, trunc={trunc}), return={ep_r:.2f}")
            obs, info = env.reset()
            ep_r = 0.0
    env.close()
    print("smoke test done.")
