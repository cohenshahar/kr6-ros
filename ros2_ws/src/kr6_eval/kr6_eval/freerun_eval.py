"""GATE R5: free-running eval — the plant never waits for the controller.

sim_node ticks a real-time 50 ms window timer and applies the LATEST joint
command; policy_node predicts on the freshest observation (depth-1 QoS, stale
frames dropped); executor_node solves IK against the freshest state. This is
the timing contract of a real robot. Measured, not assumed: control rate,
command age (obs->applied, in windows), and success over 24 episodes.

No numeric expectation (PLAN_ROS.md R5): lockstep ACT C0 is 23/24; whatever
free-run yields is the documented result.
"""
from __future__ import annotations

import json
import os
import sys
import time

VENV_SP = os.environ.get(
    "KR6_VENV_SP",
    "/home/michael/Desktop/shahar/act/.venv/lib/python3.10/site-packages")
if VENV_SP not in sys.path:
    sys.path.insert(0, VENV_SP)

import rclpy
from rclpy.node import Node

from kr6_msgs.msg import Obs
from kr6_msgs.srv import FinishEpisode, Reset

REPO = "/home/michael/Desktop/shahar/kr6-ros"
TASK, EPISODES, SEED_BASE = "box", 24, 50000
EP_TIMEOUT_S = 40.0


class FreerunClient(Node):
    def __init__(self):
        super().__init__("freerun_eval_client")
        self.cli_reset = self.create_client(Reset, "/kr6/reset")
        self.cli_finish = self.create_client(FinishEpisode, "/kr6/finish_episode")
        for c in (self.cli_reset, self.cli_finish):
            assert c.wait_for_service(timeout_sec=180.0), "sim services missing"
        self.last_obs = None
        self.create_subscription(Obs, "/kr6/obs", self._on_obs, 2)

    def _on_obs(self, msg):
        self.last_obs = msg

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=300.0)
        assert fut.done() and fut.result().ok
        return fut.result()

    def wait_episode_end(self):
        """Spin until done obs or window budget exhausted (or wall timeout)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < EP_TIMEOUT_S:
            rclpy.spin_once(self, timeout_sec=0.5)
            o = self.last_obs
            if o is not None and (o.done or o.window >= 350):
                # small grace so the sim timer parks the episode state
                time.sleep(0.2)
                return
        raise TimeoutError("episode wall timeout")


def main():
    rclpy.init()
    cli = FreerunClient()
    episodes = []
    for ep in range(EPISODES):
        t0 = time.time()
        cli.last_obs = None
        r = cli.call(cli.cli_reset, Reset.Request(
            seed=SEED_BASE + ep, task=TASK, condition=""))
        info = json.loads(r.info)
        cli.wait_episode_end()
        f = cli.call(cli.cli_finish, FinishEpisode.Request(
            video_prefix=f"{REPO}/results/r5_videos/act_s00_{TASK}_ep{ep}"))
        st = json.loads(f.stats_json) if f.stats_json else {}
        st.update(ep=ep, ep_seed=info["ep_seed_final"],
                  wall_s=round(time.time() - t0, 1))
        episodes.append(st)
        print(f"ep{ep:02d}: {'PASS' if st.get('success') else 'FAIL'} "
              f"windows={st.get('windows')} applied={st.get('applied_windows')} "
              f"age_mean={st.get('cmd_age_mean_windows')}w "
              f"wall_ms/win={st.get('window_wall_ms_mean')} "
              f"({st['wall_s']}s)", flush=True)
    n_ok = sum(1 for e in episodes if e.get("success"))
    ages = [e["cmd_age_mean_windows"] for e in episodes
            if e.get("cmd_age_mean_windows") is not None]
    report = dict(
        gate="R5", mode="freerun", window_period_s=0.05,
        success_total=f"{n_ok}/{EPISODES}",
        lockstep_reference="23/24 (R3)",
        cmd_age_mean_windows_overall=(round(sum(ages) / len(ages), 2)
                                      if ages else None),
        episodes=episodes)
    os.makedirs(f"{REPO}/results", exist_ok=True)
    json.dump(report, open(f"{REPO}/results/r5_freerun.json", "w"), indent=1)
    print(f"R5 done: {report['success_total']} free-run "
          f"(lockstep 23/24), mean cmd age "
          f"{report['cmd_age_mean_windows_overall']} windows")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
