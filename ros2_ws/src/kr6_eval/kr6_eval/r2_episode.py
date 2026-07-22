"""GATE R2 verification: one full ACT episode through the 3-node chain.

Chain per window (lockstep, strict order):
  sim.get_obs/apply_joint -> policy.get_chunk -> executor.solve_ik -> sim.apply_joint

Reference: episode 0 of act/results/nt17_gen/nt17_eval_act_C0_base.json
(system 0, task box, seed 50000 -> substitute 1050000, success @ window 163).
Computes the exact metric set of eval_act.main and diffs against the reference.
Writes results/r2_first_episode.json; exit 0 iff success and windows within ±5.
"""
from __future__ import annotations

import json
import os
import sys

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
REF_JSON = "/home/michael/Desktop/shahar/act/results/nt17_gen/nt17_eval_act_C0_base.json"
BASE_XY = np.array([-0.45, -0.60])
SEED, TASK, MAX_WINDOWS = 50000, "box", 350


class R2Client(Node):
    def __init__(self):
        super().__init__("r2_episode_client")
        self.cli = {
            "reset": self.create_client(Reset, "/kr6/reset"),
            "obs": self.create_client(GetObs, "/kr6/get_obs"),
            "chunk": self.create_client(GetChunk, "/kr6/get_chunk"),
            "ik": self.create_client(SolveIK, "/kr6/solve_ik"),
            "joint": self.create_client(ApplyJoint, "/kr6/apply_joint"),
            "finish": self.create_client(FinishEpisode, "/kr6/finish_episode"),
        }
        for name, c in self.cli.items():
            assert c.wait_for_service(timeout_sec=120.0), f"service {name} missing"

    def call(self, name, req):
        fut = self.cli[name].call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=600.0)
        assert fut.done(), f"{name} timed out"
        r = fut.result()
        assert getattr(r, "ok", True), f"{name} returned ok=False"
        return r


def flange(o):
    return np.array([o.flange_pose.position.x, o.flange_pose.position.y,
                     o.flange_pose.position.z])


def obj_pos(o):
    return np.array([o.object_pose.position.x, o.object_pose.position.y,
                     o.object_pose.position.z])


def main():
    ref = json.load(open(REF_JSON))["episodes"][0]
    rclpy.init()
    cli = R2Client()

    r = cli.call("reset", Reset.Request(seed=SEED, task=TASK, condition=""))
    info = json.loads(r.info)
    instruction = info["instruction"]
    obj_start = info["obj_start"]

    o = cli.call("obs", GetObs.Request())
    vac_prev = 0.0
    min_d = float(np.linalg.norm(flange(o) - obj_pos(o)))
    min_d_win = -1
    acts = []
    success, success_win = False, None
    for w in range(MAX_WINDOWS):
        c = cli.call("chunk", GetChunk.Request(
            obs_seq=o.obs_seq, reset_queue=(w == 0), image=o.images[0],
            state=[*[float(v) for v in flange(o)], vac_prev],
            instruction=instruction))
        a = np.array(c.chunk.data, dtype=np.float32)
        acts.append([round(float(v), 5) for v in a])
        ik = cli.call("ik", SolveIK.Request(
            action=[float(v) for v in a],
            qpos_full=list(o.qpos_full), flange_pose=o.flange_pose))
        o = cli.call("joint", ApplyJoint.Request(q_wp=list(ik.q_wp),
                                                 vacuum=ik.vacuum))
        vac_prev = 1.0 if a[6] > 0.5 else 0.0
        d = float(np.linalg.norm(flange(o) - obj_pos(o)))
        if d < min_d:
            min_d, min_d_win = d, w
        if o.done:
            success, success_win = True, w
            break

    cli.call("finish", FinishEpisode.Request(
        video_prefix=f"{REPO}/results/r2_videos/act_s00_{TASK}_ep0"))

    acts_np = np.array(acts)
    net = acts_np[:, :2].sum(axis=0)
    obj_v = np.array(obj_start[:2]) - BASE_XY
    nn, on = np.linalg.norm(net), np.linalg.norm(obj_v)
    got = dict(
        success=success, success_window=success_win, windows=len(acts),
        min_ee_obj_dist_m=round(min_d, 4), min_ee_obj_dist_window=min_d_win,
        obj_start=obj_start,
        obj_final=[round(float(v), 4) for v in obj_pos(o)],
        obj_moved_m=round(float(np.linalg.norm(
            obj_pos(o)[:2] - np.array(obj_start[:2]))), 4),
        action_abs_mean=[round(float(v), 5) for v in np.abs(acts_np).mean(axis=0)],
        vacuum_on_frac=round(float((acts_np[:, 6] > 0.5).mean()), 3),
        net_cmd_xy_mm=[round(float(v) * 1000, 1) for v in net],
        cos_cmd_vs_obj=round(float(net @ obj_v / (nn * on)), 3),
        ep_seed=info["ep_seed_final"])

    keys = ["success", "success_window", "windows", "min_ee_obj_dist_m",
            "min_ee_obj_dist_window", "obj_start", "obj_final", "obj_moved_m",
            "action_abs_mean", "vacuum_on_frac", "net_cmd_xy_mm",
            "cos_cmd_vs_obj", "ep_seed"]
    diff = {k: dict(ros=got.get(k), ref=ref.get(k),
                    match=got.get(k) == ref.get(k)) for k in keys}
    gate_pass = success and abs((success_win or 10**6) -
                                ref["success_window"]) <= 5
    report = dict(gate="R2", gate_pass=gate_pass, diff=diff)
    os.makedirs(f"{REPO}/results", exist_ok=True)
    json.dump(report, open(f"{REPO}/results/r2_first_episode.json", "w"),
              indent=1)
    exact = sum(d["match"] for d in diff.values())
    print(f"R2 {'PASS' if gate_pass else 'FAIL'}: success={success} "
          f"win={success_win} (ref {ref['success_window']}) | "
          f"exact-match {exact}/{len(keys)} fields")
    for k, d in diff.items():
        if not d["match"]:
            print(f"  drift {k}: ros={d['ros']} ref={d['ref']}")
    rclpy.shutdown()
    sys.exit(0 if gate_pass else 1)


if __name__ == "__main__":
    main()
