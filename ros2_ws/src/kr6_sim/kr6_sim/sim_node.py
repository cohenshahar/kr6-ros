"""sim_node — MuJoCo scene_v2 as a ROS2 node (GATE R1).

Wraps the frozen nt16 pipeline modules (core.SceneV2, oracle.setup_episode,
executor.preroll/execute_window, visibility.layout_ok) WITHOUT copying them —
zero code drift from the pipeline that produced the reference numbers
(ACT C0 23/24). The node owns model+data, physics stepping, episode setup,
success predicates, and video recording (policy cam + wide spectator).

Lockstep services (called by eval_node in strict order, one at a time):
  /kr6/reset          (kr6_msgs/Reset)         seeded episode setup + preroll
  /kr6/get_obs        (kr6_msgs/GetObs)        window-0 observation, no stepping
  /kr6/apply_cmd      (kr6_msgs/ApplyCmd)      one action window + fresh obs
  /kr6/finish_episode (kr6_msgs/FinishEpisode) flush videos to disk
"""
from __future__ import annotations

import json
import os
import sys

# ── python-path bridge (system rclpy + act-venv torch/mujoco coexist) ──────
VENV_SP = os.environ.get(
    "KR6_VENV_SP",
    "/home/michael/Desktop/shahar/act/.venv/lib/python3.10/site-packages")
NT16_DIR = os.environ.get(
    "KR6_NT16_DIR", "/home/michael/Desktop/shahar/vla-sim-playground/nt16")
for p in (VENV_SP, NT16_DIR, os.path.dirname(NT16_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Pose  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402

from kr6_msgs.msg import JointCmd, Obs  # noqa: E402
from kr6_msgs.srv import ApplyCmd, ApplyJoint, FinishEpisode, GetObs, Reset  # noqa: E402

from core import (ARM, EXEC_GATE, EXEC_OVERTIME, LIFT_HEIGHT,  # noqa: E402
                  LIFT_HOLD_WINDOWS, TASKS, WINDOW_STEPS, SceneV2, load_systems)
from executor import execute_window, preroll  # noqa: E402
from oracle import setup_episode  # noqa: E402
from visibility import MIN_VISIBLE_PX, layout_ok  # noqa: E402

SPEC_CAM = "spectator_close"
SPEC_RES = (480, 640)  # h, w
EVAL_SEED_BASE = 50000  # eval_act.py convention, kept for seed substitution


def _np_to_image_msg(arr, stamp, frame_id):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = arr.shape[0], arr.shape[1]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = arr.shape[1] * 3
    msg.data = arr.tobytes()
    return msg


def _pose_from_xyz(p):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, p)
    msg.orientation.w = 1.0
    return msg


class SimNode(Node):
    def __init__(self):
        super().__init__("sim_node")
        self.declare_parameter("system_id", 0)
        self.declare_parameter("render", 224)
        self.declare_parameter("obs_cameras", [""])  # extra named cams (3cam)
        self.declare_parameter("mode", "lockstep")   # lockstep | freerun
        self.declare_parameter("window_period_s", 0.05)  # freerun: real-time 50 ms
        sysid = int(self.get_parameter("system_id").value)
        render = int(self.get_parameter("render").value)
        self.obs_cameras = [c for c in
                            self.get_parameter("obs_cameras").value if c]

        self.systems = load_systems()
        self.system = self.systems[sysid]
        self.scene = SceneV2(self.system, render=render)
        import mujoco
        self.spec_r = mujoco.Renderer(self.scene.m, height=SPEC_RES[0],
                                      width=SPEC_RES[1])
        # episode state
        self.task = None
        self.obj = None
        self.rest_z = None
        self.hold = 0
        self.obs_seq = 0
        self.frames = []       # policy-cam frames
        self.spec_frames = []  # spectator frames

        self.create_service(Reset, "/kr6/reset", self.on_reset)
        self.create_service(GetObs, "/kr6/get_obs", self.on_get_obs)
        self.create_service(ApplyCmd, "/kr6/apply_cmd", self.on_apply_cmd)
        self.create_service(ApplyJoint, "/kr6/apply_joint", self.on_apply_joint)
        self.create_service(FinishEpisode, "/kr6/finish_episode", self.on_finish)
        # debug/freerun topics (best-effort mirrors of the service payloads)
        self.pub_img = self.create_publisher(Image, "/kr6/camera/policy", 2)
        self.pub_js = self.create_publisher(JointState, "/kr6/joint_states", 2)
        # ── free-run mode: timer-driven windows, latest-wins joint command ──
        self.mode = str(self.get_parameter("mode").value)
        self.window = 0
        self.latest_cmd = None
        self.episode_active = False
        self.stats = None
        if self.mode == "freerun":
            import time as _time
            self._time = _time
            period = float(self.get_parameter("window_period_s").value)
            self.pub_obs = self.create_publisher(Obs, "/kr6/obs", 2)
            self.create_subscription(JointCmd, "/kr6/joint_cmd",
                                     self.on_joint_cmd, 2)
            self.create_timer(period, self.on_window_timer)
        self.get_logger().info(
            f"sim_node up: system {sysid} (camera {self.system['camera']}), "
            f"render {render}, extra obs cams {self.obs_cameras}, "
            f"nt16 at {NT16_DIR}")

    # ── helpers ────────────────────────────────────────────────────────────
    def _render_policy(self):
        s = self.scene
        s.r.update_scene(s.d, camera=s.camera)
        return np.asarray(s.r.render(), dtype=np.uint8)

    def _render_spec(self):
        self.spec_r.update_scene(self.scene.d, camera=SPEC_CAM)
        return np.asarray(self.spec_r.render(), dtype=np.uint8)

    def _success(self):
        s = self.scene
        if self.task == "box_out":
            return s.on_table_outside_crate(self.obj)
        if self.task.startswith("lift"):
            self.hold = (self.hold + 1
                         if float(s.obj_pos(self.obj)[2]) > self.rest_z + LIFT_HEIGHT
                         else 0)
            return self.hold >= LIFT_HOLD_WINDOWS
        return s.in_crate(self.obj)

    def _fill_obs(self, res, frame, done):
        s = self.scene
        self.obs_seq += 1
        res.ok = True
        res.obs_seq = self.obs_seq
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = list(ARM)
        js.position = [float(v) for v in s.d.qpos[s.qadr]]
        res.joint_state = js
        res.qpos_full = [float(v) for v in s.d.qpos]
        res.flange_pose = _pose_from_xyz(s.d.site_xpos[s.fid])
        res.object_pose = _pose_from_xyz(s.obj_pos(self.obj))
        img = _np_to_image_msg(frame, stamp, self.system["camera"])
        res.images = [img]
        for cam in self.obs_cameras:  # same renderer re-posed (eval_smolvla)
            s.r.update_scene(s.d, camera=cam)
            extra = np.asarray(s.r.render(), dtype=np.uint8)
            res.images.append(_np_to_image_msg(extra, stamp, cam))
        res.done = bool(done)
        self.pub_img.publish(img)
        self.pub_js.publish(js)
        return res

    # ── services ───────────────────────────────────────────────────────────
    def on_reset(self, req, res):
        try:
            if req.condition:
                res.ok = False
                res.info = "conditions (nt17) land at the reserve gate; use C0"
                return res
            task = req.task or "box"
            ep_seed, vis_tries, vis_px = int(req.seed), 0, -1
            # visibility gate exactly as eval_act.main (substitute-seed rule)
            while True:
                setup_episode(self.scene, self.system, ep_seed, task)
                ok_vis, vis_px = layout_ok(self.scene, TASKS[task]["obj"],
                                           min_px=MIN_VISIBLE_PX)
                if ok_vis or vis_tries >= 20:
                    break
                vis_tries += 1
                base_ep = int(req.seed) - EVAL_SEED_BASE
                ep_seed = EVAL_SEED_BASE + 1_000_000 * vis_tries + base_ep
            preroll(self.scene)
            self.task, self.obj = task, TASKS[task]["obj"]
            obj_start = self.scene.obj_pos(self.obj).tolist()
            self.rest_z = float(obj_start[2])
            self.hold = 0
            self.frames, self.spec_frames = [], []
            if self.mode == "freerun":
                self.window = 0
                self.latest_cmd = None
                self.stats = dict(applied_windows=0, cmd_age_windows=[],
                                  window_wall_ms=[], obs_window_of={},
                                  max_windows=350, success=False, windows=0)
                self.stats["obs_window_of"][self.obs_seq + 1] = 0
                self._publish_obs(False)
                self.episode_active = True
            res.ok = True
            res.info = json.dumps(dict(
                ep_seed_final=ep_seed, vis_tries=vis_tries, vis_px=int(vis_px),
                instruction=TASKS[task]["instruction"],
                obj_start=[round(v, 4) for v in obj_start]))
        except Exception as e:  # surface the real error to the caller
            self.get_logger().error(f"reset failed: {e!r}")
            res.ok = False
            res.info = repr(e)
        return res

    def on_get_obs(self, req, res):
        frame = self._render_policy()
        self.frames.append(frame)
        self.spec_frames.append(self._render_spec())
        return self._fill_obs(res, frame, done=False)

    def on_apply_cmd(self, req, res):
        try:
            a = np.asarray(req.cmd, dtype=np.float32)
            assert a.shape == (7,), f"cmd must be 7-D, got {a.shape}"
            execute_window(self.scene, a)
            done = self._success()
            frame = self._render_policy()
            self.frames.append(frame)
            self.spec_frames.append(self._render_spec())
            return self._fill_obs(res, frame, done)
        except Exception as e:
            self.get_logger().error(f"apply_cmd failed: {e!r}")
            res.ok = False
            return res

    def on_apply_joint(self, req, res):
        """Plant-side half of execute_window: vacuum + lead-gain joint servo.
        Byte-identical loop to nt16 execute_window after its IK line."""
        try:
            import mujoco
            s = self.scene
            q_wp = np.asarray(req.q_wp, dtype=float)
            assert q_wp.shape == (6,), f"q_wp must be 6-D, got {q_wp.shape}"
            s.d.ctrl[s.vac] = 1.0 if req.vacuum > 0.5 else 0.0
            for _ in range(WINDOW_STEPS * EXEC_OVERTIME):
                e = q_wp - np.array(s.d.qpos[s.qadr])
                if float(np.max(np.abs(e))) < EXEC_GATE:
                    break
                s.d.ctrl[s.act] = np.clip(q_wp + 2.0 * e, s.lo, s.hi)
                mujoco.mj_step(s.m, s.d)
            s.d.ctrl[s.act] = q_wp
            done = self._success()
            frame = self._render_policy()
            self.frames.append(frame)
            self.spec_frames.append(self._render_spec())
            return self._fill_obs(res, frame, done)
        except Exception as e:
            self.get_logger().error(f"apply_joint failed: {e!r}")
            res.ok = False
            return res

    def on_joint_cmd(self, msg):
        self.latest_cmd = msg

    def _publish_obs(self, done):
        s = self.scene
        self.obs_seq += 1
        m = Obs()
        m.obs_seq = self.obs_seq
        m.window = self.window
        stamp = self.get_clock().now().to_msg()
        m.stamp = stamp
        js = JointState()
        js.header.stamp = stamp
        js.name = list(ARM)
        js.position = [float(v) for v in s.d.qpos[s.qadr]]
        m.joint_state = js
        m.qpos_full = [float(v) for v in s.d.qpos]
        m.flange_pose = _pose_from_xyz(s.d.site_xpos[s.fid])
        m.object_pose = _pose_from_xyz(s.obj_pos(self.obj))
        frame = self._render_policy()
        self.frames.append(frame)
        self.spec_frames.append(self._render_spec())
        m.images = [_np_to_image_msg(frame, stamp, self.system["camera"])]
        for cam in self.obs_cameras:
            s.r.update_scene(s.d, camera=cam)
            extra = np.asarray(s.r.render(), dtype=np.uint8)
            m.images.append(_np_to_image_msg(extra, stamp, cam))
        m.done = bool(done)
        self.pub_obs.publish(m)

    def on_window_timer(self):
        """One real-time control window: apply the LATEST joint command (or
        hold), step the servo loop, publish a fresh observation. The plant
        never waits for the controller — that is the free-run contract."""
        if not self.episode_active:
            return
        import mujoco
        s = self.scene
        t0 = self._time.monotonic()
        cmd = self.latest_cmd
        if cmd is not None:
            q_wp = np.asarray(cmd.q_wp, dtype=float)
            s.d.ctrl[s.vac] = 1.0 if cmd.vacuum > 0.5 else 0.0
            for _ in range(WINDOW_STEPS * EXEC_OVERTIME):
                e = q_wp - np.array(s.d.qpos[s.qadr])
                if float(np.max(np.abs(e))) < EXEC_GATE:
                    break
                s.d.ctrl[s.act] = np.clip(q_wp + 2.0 * e, s.lo, s.hi)
                mujoco.mj_step(s.m, s.d)
            s.d.ctrl[s.act] = q_wp
            self.stats["applied_windows"] += 1
            self.stats["cmd_age_windows"].append(
                int(self.window) - self.stats["obs_window_of"].get(
                    cmd.obs_seq, int(self.window)))
        else:
            for _ in range(WINDOW_STEPS):   # no command yet: physics idles on
                mujoco.mj_step(s.m, s.d)
        self.window += 1
        done = self._success()
        self.stats["obs_window_of"][self.obs_seq + 1] = int(self.window)
        self._publish_obs(done)
        self.stats["window_wall_ms"].append(
            round((self._time.monotonic() - t0) * 1000, 2))
        if done or self.window >= self.stats["max_windows"]:
            self.episode_active = False
            self.stats["success"] = bool(done)
            self.stats["windows"] = int(self.window)

    def on_finish(self, req, res):
        try:
            if req.video_prefix:
                import imageio.v2 as imageio
                os.makedirs(os.path.dirname(req.video_prefix), exist_ok=True)
                res.policy_video = req.video_prefix + ".mp4"
                res.spec_video = req.video_prefix + "_spec.mp4"
                imageio.mimsave(res.policy_video, self.frames, fps=8)
                imageio.mimsave(res.spec_video, self.spec_frames, fps=8)
            self.frames, self.spec_frames = [], []
            if self.mode == "freerun" and self.stats is not None:
                st = dict(self.stats)
                st.pop("obs_window_of", None)
                ages = st.pop("cmd_age_windows", [])
                walls = st.pop("window_wall_ms", [])
                st["cmd_age_mean_windows"] = (round(float(np.mean(ages)), 2)
                                              if ages else None)
                st["cmd_age_max_windows"] = int(np.max(ages)) if ages else None
                st["window_wall_ms_mean"] = (round(float(np.mean(walls)), 2)
                                             if walls else None)
                res.stats_json = json.dumps(st)
            res.ok = True
        except Exception as e:
            self.get_logger().error(f"finish_episode failed: {e!r}")
            res.ok = False
        return res


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
