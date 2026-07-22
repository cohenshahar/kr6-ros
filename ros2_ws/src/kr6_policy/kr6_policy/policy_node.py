"""policy_node — lerobot policy (ACT / SmolVLA) as a ROS2 service (GATE R2).

Loads the checkpoint once (CUDA), then serves /kr6/get_chunk: one observation
in -> one 7-D action out, using eval_act.predict verbatim (the policy's
internal chunk queue re-predicts every n_action_steps). reset_queue=True on
the first call of an episode maps to policy.reset() — same call sequence as
the direct eval loop.

Parameters:
  policy       "act" (default) | "smolvla"
  checkpoint   pretrained_model dir (default: the nt17 reference checkpoints)
"""
from __future__ import annotations

import os
import sys

VENV_SP = os.environ.get(
    "KR6_VENV_SP",
    "/home/michael/Desktop/shahar/act/.venv/lib/python3.10/site-packages")
ACT_DIR = os.environ.get("KR6_ACT_DIR", "/home/michael/Desktop/shahar/act")
NT16_DIR = os.environ.get(
    "KR6_NT16_DIR", "/home/michael/Desktop/shahar/vla-sim-playground/nt16")
for p in (VENV_SP, ACT_DIR, NT16_DIR, os.path.dirname(NT16_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from kr6_msgs.msg import ActionChunk  # noqa: E402
from kr6_msgs.srv import GetChunk  # noqa: E402

DEFAULT_CKPT = {
    "act": "/home/michael/Desktop/shahar/act/runs/act_boxv2/checkpoints/last/pretrained_model",
    "smolvla": "/home/michael/Desktop/shahar/act/runs/smolvla_boxv2_3cam/checkpoints/last/pretrained_model",
}


class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy_node")
        self.declare_parameter("policy", "act")
        self.declare_parameter("checkpoint", "")
        kind = str(self.get_parameter("policy").value)
        ckpt = str(self.get_parameter("checkpoint").value) or DEFAULT_CKPT[kind]
        self.kind = kind
        if kind == "act":
            from eval_act import load_policy, predict
            self.cam_map = None                    # single frame, positional
        else:
            from eval_smolvla import (CAM_MAP_3, CAM_MAP_SINGLE,
                                      detect_camera_mode, load_policy, predict)
            mode = detect_camera_mode(ckpt)
            self.cam_map = CAM_MAP_SINGLE if mode == "single" else CAM_MAP_3
            self.get_logger().info(f"smolvla camera mode: {mode}")
        self.predict = predict
        self.policy, self.pre, self.post = load_policy(ckpt)
        self.create_service(GetChunk, "/kr6/get_chunk", self.on_get_chunk)
        self.get_logger().info(f"policy_node up: {kind} @ {ckpt}")

    def on_get_chunk(self, req, res):
        try:
            if req.reset_queue:
                self.policy.reset()
            frames = {}
            for img in req.images:
                assert img.encoding == "rgb8", f"bad encoding {img.encoding}"
                frames[img.header.frame_id] = np.frombuffer(
                    bytes(img.data), dtype=np.uint8).reshape(
                    img.height, img.width, 3)
            sys_frame = np.frombuffer(
                bytes(req.images[0].data), dtype=np.uint8).reshape(
                req.images[0].height, req.images[0].width, 3)
            state = np.asarray(req.state, dtype=np.float32)
            if self.cam_map is None:               # ACT: positional frame
                a = self.predict(self.policy, self.pre, self.post, sys_frame,
                                 state, req.instruction)
            else:                                  # SmolVLA: dict of cameras
                imgs = {key: (sys_frame if cam is None else frames[cam])
                        for key, cam in self.cam_map.items()}
                a = self.predict(self.policy, self.pre, self.post, imgs,
                                 state, req.instruction)
            chunk = ActionChunk()
            chunk.obs_seq = req.obs_seq
            chunk.chunk_len = 1
            chunk.dim = 7
            chunk.data = [float(v) for v in a]
            res.ok = True
            res.chunk = chunk
        except Exception as e:
            self.get_logger().error(f"get_chunk failed: {e!r}")
            res.ok = False
        return res


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
