"""Bring up the full 3-node KR6 system (+ eval orchestrator added at R2)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='kr6_sim', executable='sim_node', name='sim_node'),
        Node(package='kr6_policy', executable='policy_node', name='policy_node'),
        Node(package='kr6_executor', executable='executor_node', name='executor_node'),
    ])
