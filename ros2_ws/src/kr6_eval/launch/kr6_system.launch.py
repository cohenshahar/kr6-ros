"""Bring up the full 3-node KR6 system.

All nodes run under the act venv interpreter: it bundles torch/mujoco/lerobot
AND sees /opt/ros/humble (baked into the venv path), while excluding the
user-site packages that clash (sklearn/scipy vs venv numpy — GATE R2 finding).
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess

VP = "/home/michael/Desktop/shahar/act/.venv/bin/python"


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=[VP, "-m", "kr6_sim.sim_node"],
                       name="sim_node", output="screen"),
        ExecuteProcess(cmd=[VP, "-m", "kr6_executor.executor_node"],
                       name="executor_node", output="screen"),
        ExecuteProcess(cmd=[VP, "-m", "kr6_policy.policy_node"],
                       name="policy_node", output="screen"),
    ])
