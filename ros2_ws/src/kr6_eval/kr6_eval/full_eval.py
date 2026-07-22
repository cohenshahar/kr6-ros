"""Generalized full-eval client through the 3-node ROS chain (GATE R4+).

Same protocol as r3_full_eval (which stays frozen as R3 evidence), with the
policy, reference JSON, and output paths as arguments. Sends ALL obs images
to the policy node — policy_node maps them by frame_id (3cam SmolVLA) or uses
the system frame (ACT).

  .venv python full_eval.py --gate R4 \
      --ref .../nt17_eval_smolvla_C0_base.json \
      --out results/r4_smolvla_c0_ros.json --videos results/r4_videos \
      --tag smolvla
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

VENV_SP = os.environ.get(
    "KR6_VENV_SP",
    "/home/michael/Desktop/shahar/act/.venv/lib/python3.10/site-packages")
if VENV_SP not in sys.path:
    sys.path.insert(0, VENV_SP)

import numpy as np
import rclpy
from rclpy.node import Node

from kr6_msgs.srv import ApplyJoint, FinishEpisode, GetChunk, GetObs, Reset, SolveIK

REPO = "/home/michael/Desktop/shahar/kr6-ros"
BASE_XY = np.array([-0.45, -0.60])
SEED_BASE = 50000
COMPARE_KEYS = ["success", "success_window", "windows", "min_ee_obj_dist_m",
                "min_ee_obj_dist_window", "obj_start", "obj_final",
                "obj_moved_m", "action_abs_mean", "vacuum_on_frac",
                "net_cmd_xy_mm", "cos_cmd_vs_obj", "ep_seed"]


class Client(Node):
    def __init__(self):
        super().__init__("full_eval_client")
        self.cli = {
            "reset": self.create_client(Reset, "/kr6/reset"),
            "obs": self.create_client(GetObs, "/kr6/get_obs"),
            "chunk": self.create_client(GetChunk, "/kr6/get_chunk"),
            "ik": self.create_client(SolveIK, "/kr6/solve_ik"),
            "joint": self.create_client(ApplyJoint, "/kr6/apply_joint"),
            "finish": self.create_client(FinishEpisode, "/kr6/finish_episode"),
        }
        for name, c in self.cli.items():
            assert c.wait_for_service(timeout_sec=300.0), f"service {name} missing"

    def call(self, name, req, timeout=900.0):
        fut = self.cli[name].call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        assert fut.done(), f"{name} timed out"
        r = fut.result()
        assert getattr(r, "ok", True), f"{name} returned ok=False"
        return r


def _xyz(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


def run_episode(cli, ep, task, max_windows, videos, tag):
    t0 = time.time()
    r = cli.call("reset", Reset.Request(seed=SEED_BASE + ep, task=task,
                                        condition=""))
    info = json.loads(r.info)
    o = cli.call("obs", GetObs.Request())
    vac_prev, acts = 0.0, []
    min_d = float(np.linalg.norm(_xyz(o.flange_pose) - _xyz(o.object_pose)))
    min_d_win = -1
    success, success_win = False, None
    for w in range(max_windows):
        c = cli.call("chunk", GetChunk.Request(
            obs_seq=o.obs_seq, reset_queue=(w == 0), images=list(o.images),
            state=[*[float(v) for v in _xyz(o.flange_pose)], vac_prev],
            instruction=info["instruction"]))
        a = np.array(c.chunk.data, dtype=np.float32)
        acts.append([round(float(v), 5) for v in a])
        ik = cli.call("ik", SolveIK.Request(
            action=[float(v) for v in a], qpos_full=list(o.qpos_full),
            flange_pose=o.flange_pose))
        o = cli.call("joint", ApplyJoint.Request(q_wp=list(ik.q_wp),
                                                 vacuum=ik.vacuum))
        vac_prev = 1.0 if a[6] > 0.5 else 0.0
        d = float(np.linalg.norm(_xyz(o.flange_pose) - _xyz(o.object_pose)))
        if d < min_d:
            min_d, min_d_win = d, w
        if o.done:
            success, success_win = True, w
            break
    cli.call("finish", FinishEpisode.Request(
        video_prefix=f"{videos}/{tag}_s00_{task}_ep{ep}"))
    acts_np = np.array(acts)
    net = acts_np[:, :2].sum(axis=0)
    obj_v = np.array(info["obj_start"][:2]) - BASE_XY
    nn, on = np.linalg.norm(net), np.linalg.norm(obj_v)
    return dict(
        success=success, success_window=success_win, windows=len(acts),
        min_ee_obj_dist_m=round(min_d, 4), min_ee_obj_dist_window=min_d_win,
        obj_start=info["obj_start"],
        obj_final=[round(float(v), 4) for v in _xyz(o.object_pose)],
        obj_moved_m=round(float(np.linalg.norm(
            _xyz(o.object_pose)[:2] - np.array(info["obj_start"][:2]))), 4),
        action_abs_mean=[round(float(v), 5)
                         for v in np.abs(acts_np).mean(axis=0)],
        vacuum_on_frac=round(float((acts_np[:, 6] > 0.5).mean()), 3),
        net_cmd_xy_mm=[round(float(v) * 1000, 1) for v in net],
        cos_cmd_vs_obj=(round(float(net @ obj_v / (nn * on)), 3)
                        if nn > 1e-9 and on > 1e-9 else None),
        ep_seed=info["ep_seed_final"], wall_s=round(time.time() - t0, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gate", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--videos", required=True)
    ap.add_argument("--tag", default="policy")
    ap.add_argument("--task", default="box")
    ap.add_argument("--episodes", type=int, default=24)
    ap.add_argument("--max-windows", type=int, default=350)
    args = ap.parse_args()

    ref = json.load(open(args.ref))
    rclpy.init()
    cli = Client()
    episodes, drift_rows = [], []
    for ep in range(args.episodes):
        got = run_episode(cli, ep, args.task, args.max_windows,
                          args.videos, args.tag)
        re_ = ref["episodes"][ep]
        drift = {k: dict(ros=got.get(k), ref=re_.get(k))
                 for k in COMPARE_KEYS if got.get(k) != re_.get(k)}
        got["exact_match"] = not drift
        got["success_match"] = got["success"] == re_["success"]
        if drift:
            drift_rows.append({"ep": ep, **drift})
        episodes.append(got)
        print(f"ep{ep:02d}: {'PASS' if got['success'] else 'FAIL'} "
              f"win={got['success_window']} "
              f"{'EXACT' if got['exact_match'] else 'DRIFT ' + str(list(drift))} "
              f"({got['wall_s']}s)", flush=True)
    n_ok = sum(e["success"] for e in episodes)
    total = f"{n_ok}/{args.episodes}"
    gate_pass = (total == ref["success_total"]
                 and all(e["success_match"] for e in episodes))
    report = dict(gate=args.gate, gate_pass=gate_pass, success_total=total,
                  ref_success_total=ref["success_total"],
                  exact_episodes=sum(e["exact_match"] for e in episodes),
                  drift=drift_rows, episodes=episodes)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    json.dump(report, open(args.out, "w"), indent=1)
    print(f"{args.gate} {'PASS' if gate_pass else 'FAIL'}: {total} "
          f"(ref {ref['success_total']}), exact "
          f"{report['exact_episodes']}/{args.episodes}")
    rclpy.shutdown()
    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
