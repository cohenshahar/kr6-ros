"""GATE R1 verification client.

Runs against a live sim_node and checks, for several seeds:
  1. reset(seed) via ROS returns the SAME episode layout as calling the nt16
     code directly (obj_start parity, atol 1e-9 — same code, same numbers).
  2. get_obs returns a valid 224x224 rgb8 frame and a 6-joint state.
  3. A short scripted action sequence runs through apply_cmd and the flange
     actually moves (executor alive through the service boundary).
  4. finish_episode writes both videos (policy + _spec).

Writes results/r1_reset_parity.json in the repo root. Exit 0 iff all pass.
"""
from __future__ import annotations

import json
import os
import sys

VENV_SP = os.environ.get(
    "KR6_VENV_SP",
    "/home/michael/Desktop/shahar/act/.venv/lib/python3.10/site-packages")
NT16_DIR = os.environ.get(
    "KR6_NT16_DIR", "/home/michael/Desktop/shahar/vla-sim-playground/nt16")
for p in (VENV_SP, NT16_DIR, os.path.dirname(NT16_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import rclpy
from rclpy.node import Node

from kr6_msgs.srv import ApplyCmd, FinishEpisode, GetObs, Reset

REPO = "/home/michael/Desktop/shahar/kr6-ros"
SEEDS = [50000, 50007, 50012, 51003]
TASK = "box"
# small scripted probe: 6 windows of 8 mm descent + vacuum on
PROBE = [np.array([0.0, 0.0, -0.008, 0.0, 0.0, 0.0, 1.0])] * 6


class R1Client(Node):
    def __init__(self):
        super().__init__("r1_parity_client")
        self.cli = {
            "reset": self.create_client(Reset, "/kr6/reset"),
            "obs": self.create_client(GetObs, "/kr6/get_obs"),
            "apply": self.create_client(ApplyCmd, "/kr6/apply_cmd"),
            "finish": self.create_client(FinishEpisode, "/kr6/finish_episode"),
        }
        for name, c in self.cli.items():
            assert c.wait_for_service(timeout_sec=30.0), f"service {name} missing"

    def call(self, name, req):
        fut = self.cli[name].call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=600.0)
        assert fut.done(), f"{name} timed out"
        return fut.result()


def direct_layouts():
    """Ground truth: same seeds through the nt16 code directly (no ROS)."""
    from core import TASKS, SceneV2, load_systems
    from oracle import setup_episode
    from visibility import MIN_VISIBLE_PX, layout_ok
    system = load_systems()[0]
    scene = SceneV2(system)
    out = {}
    for seed in SEEDS:
        ep_seed, tries = seed, 0
        while True:
            setup_episode(scene, system, ep_seed, TASK)
            ok, _ = layout_ok(scene, TASKS[TASK]["obj"], min_px=MIN_VISIBLE_PX)
            if ok or tries >= 20:
                break
            tries += 1
            ep_seed = 50000 + 1_000_000 * tries + (seed - 50000)
        out[seed] = dict(ep_seed_final=ep_seed,
                         obj_start=[round(float(v), 4)
                                    for v in scene.obj_pos(TASKS[TASK]["obj"])])
    return out


def main():
    truth = direct_layouts()
    rclpy.init()
    cli = R1Client()
    report = {"task": TASK, "seeds": [], "all_pass": True}

    for seed in SEEDS:
        row = {"seed": seed}
        r = cli.call("reset", Reset.Request(seed=seed, task=TASK, condition=""))
        assert r.ok, f"reset({seed}) failed: {r.info}"
        info = json.loads(r.info)
        row["ros"] = info
        row["direct"] = truth[seed]
        row["seed_match"] = info["ep_seed_final"] == truth[seed]["ep_seed_final"]
        row["layout_match"] = bool(np.allclose(
            info["obj_start"], truth[seed]["obj_start"], atol=1e-9))

        o = cli.call("obs", GetObs.Request())
        img = o.images[0]
        row["frame_ok"] = (img.height == 224 and img.width == 224
                           and img.encoding == "rgb8"
                           and len(img.data) == 224 * 224 * 3)
        row["joints_ok"] = len(o.joint_state.position) == 6
        z0 = o.flange_pose.position.z

        for a in PROBE:
            o = cli.call("apply", ApplyCmd.Request(cmd=[float(v) for v in a]))
            assert o.ok, "apply_cmd failed"
        moved = z0 - o.flange_pose.position.z
        row["flange_moved_mm"] = round(moved * 1000, 2)
        row["executor_ok"] = moved > 0.02  # commanded 48 mm; >20 mm proves motion

        f = cli.call("finish", FinishEpisode.Request(
            video_prefix=f"{REPO}/results/r1_videos/seed{seed}"))
        row["videos_ok"] = (f.ok and os.path.exists(f.policy_video)
                            and os.path.exists(f.spec_video)
                            and os.path.getsize(f.spec_video) > 10_000)
        row["pass"] = all(row[k] for k in
                          ("seed_match", "layout_match", "frame_ok",
                           "joints_ok", "executor_ok", "videos_ok"))
        report["all_pass"] &= row["pass"]
        report["seeds"].append(row)
        print(f"seed {seed}: {'PASS' if row['pass'] else 'FAIL'} — {row}")

    os.makedirs(f"{REPO}/results", exist_ok=True)
    with open(f"{REPO}/results/r1_reset_parity.json", "w") as fh:
        json.dump(report, fh, indent=1, ensure_ascii=False)
    rclpy.shutdown()
    print("ALL PASS" if report["all_pass"] else "FAILURES PRESENT")
    sys.exit(0 if report["all_pass"] else 1)


if __name__ == "__main__":
    main()
