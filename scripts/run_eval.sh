#!/bin/bash
# kr6-ros eval runner: clean nodes -> bring up 3-node system -> full_eval -> clean.
# Usage: run_eval.sh <act|smolvla> <gate> <ref_json> <out_json> <videos_dir> [tag]
# Process hygiene: [b]racketed pkill patterns so this script never matches itself.
POLICY=$1; GATE=$2; REF=$3; OUT=$4; VIDEOS=$5; TAG=${6:-$POLICY}
REPO=/home/michael/Desktop/shahar/kr6-ros
VP=/home/michael/Desktop/shahar/act/.venv/bin/python
LOGDIR=${KR6_LOGDIR:-/tmp}
source /opt/ros/humble/setup.bash
source $REPO/ros2_ws/install/setup.bash
set -u   # AFTER sourcing: Humble setup scripts reference unset vars

cleanup() {
  pkill -f "[k]r6_sim.sim_node" 2>/dev/null
  pkill -f "[k]r6_executor.executor_node" 2>/dev/null
  pkill -f "[k]r6_policy.policy_node" 2>/dev/null
}
cleanup; sleep 2

if [ "$POLICY" = "smolvla" ]; then
  SIM_ARGS=(--ros-args -p "obs_cameras:=[cam_lift,cam_side_ny,cam_top]")
  POL_ARGS=(--ros-args -p policy:=smolvla)
  WARMUP=120
else
  SIM_ARGS=(); POL_ARGS=(); WARMUP=75
fi
nohup $VP -m kr6_sim.sim_node "${SIM_ARGS[@]}" > $LOGDIR/eval_sim.log 2>&1 &
nohup $VP -m kr6_executor.executor_node > $LOGDIR/eval_exec.log 2>&1 &
nohup $VP -m kr6_policy.policy_node "${POL_ARGS[@]}" > $LOGDIR/eval_policy.log 2>&1 &
sleep $WARMUP

$VP $REPO/ros2_ws/src/kr6_eval/kr6_eval/full_eval.py \
  --gate "$GATE" --ref "$REF" --out "$OUT" --videos "$VIDEOS" --tag "$TAG"
RC=$?
cleanup
exit $RC
