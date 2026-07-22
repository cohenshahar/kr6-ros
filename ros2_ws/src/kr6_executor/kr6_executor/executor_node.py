"""executor_node — offline IK stage of executor v2 as a ROS2 service (GATE R2).

Holds its OWN kinematic copy of the scene (no renderer, no stepping) and runs
exactly nt16 execute_window's IK line: damped-least-squares, axis-only (cup
vertical, yaw free), flange site, EXEC_IK_ITERS iterations. Exact-state parity
is guaranteed by copying the sim's full qpos snapshot before every solve —
the same numbers execute_window sees in-process.

The plant-side half (lead-gain joint servo) stays in sim_node (/kr6/apply_joint),
mirroring a real robot: controller computes, plant executes.
"""
from __future__ import annotations

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

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from kr6_msgs.srv import SolveIK  # noqa: E402

from core import EXEC_IK_ITERS, SceneV2  # noqa: E402


class ExecutorNode(Node):
    def __init__(self):
        super().__init__("executor_node")
        # kinematics-only scene copy: no system camera, no renderer
        self.scene = SceneV2(system=None, render=0)
        self.create_service(SolveIK, "/kr6/solve_ik", self.on_solve)
        self.get_logger().info("executor_node up: kinematic scene loaded, "
                               "flange-site axis-only IK ready")

    def on_solve(self, req, res):
        try:
            s = self.scene
            a = np.asarray(req.action, dtype=np.float32)
            assert a.shape == (7,), f"action must be 7-D, got {a.shape}"
            qpos = np.asarray(req.qpos_full, dtype=float)
            assert qpos.shape[0] == s.m.nq, \
                f"qpos_full length {qpos.shape[0]} != nq {s.m.nq}"
            s.d.qpos[:] = qpos          # exact-state sync (execute_window parity)
            p0 = np.array([req.flange_pose.position.x,
                           req.flange_pose.position.y,
                           req.flange_pose.position.z])
            q_wp, _ = s.ik(p0 + np.asarray(a[0:3], dtype=float),
                           np.array(s.d.qpos[s.qadr]), iters=EXEC_IK_ITERS,
                           axis_only=True, site=s.fid)
            res.ok = True
            res.q_wp = [float(v) for v in q_wp]
            res.vacuum = 1.0 if a[6] > 0.5 else 0.0
        except Exception as e:
            self.get_logger().error(f"solve_ik failed: {e!r}")
            res.ok = False
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
