#!/bin/bash
# kr6-ros free-run (R5) runner. MUST be invoked as a script file — inline
# compound commands that both spawn nodes and pkill them match their own
# command line and kill themselves (learned twice on 22/07).
# Usage: run_freerun.sh [policy]   (default act)
POLICY=${1:-act}
REPO=/home/michael/Desktop/shahar/kr6-ros
VP=/home/michael/Desktop/shahar/act/.venv/bin/python
LOGDIR=${KR6_LOGDIR:-/tmp}
source /opt/ros/humble/setup.bash
source $REPO/ros2_ws/install/setup.bash
set -u

cleanup() {
  pkill -f "[k]r6_sim.sim_node" 2>/dev/null
  pkill -f "[k]r6_executor.executor_node" 2>/dev/null
  pkill -f "[k]r6_policy.policy_node" 2>/dev/null
}
cleanup; sleep 2

nohup $VP -m kr6_sim.sim_node --ros-args -p mode:=freerun > $LOGDIR/r5_sim.log 2>&1 &
nohup $VP -m kr6_executor.executor_node --ros-args -p mode:=freerun > $LOGDIR/r5_exec.log 2>&1 &
nohup $VP -m kr6_policy.policy_node --ros-args -p mode:=freerun -p policy:=$POLICY > $LOGDIR/r5_policy.log 2>&1 &
sleep 75

$VP $REPO/ros2_ws/src/kr6_eval/kr6_eval/freerun_eval.py
RC=$?
cleanup
exit $RC
