#!/bin/bash
# One headless run: start the cell, sort N items, grade from Gazebo's true positions, shut down.
#   bash tools/trial.sh <items> [--rate R] [--plan-margin S] [launch args...]
# --rate: simulation speed target (1 = real time, 0 = as fast as possible).
# Results: ~/ros2_ws/log/sim/tracking_result.json, sim.out, sorter.out
ITEMS=$1; shift
RATE=1; MARGIN=0.25; LA=()
while [ $# -gt 0 ]; do case "$1" in
  --rate) RATE=$2; shift 2;; --plan-margin) MARGIN=$2; shift 2;; *) LA+=("$1"); shift;; esac; done
DIR=~/ros2_ws/src/ur_pick_place; OUT=~/ros2_ws/log/sim; mkdir -p $OUT
export UR_WRIST_CAMERA=${UR_WRIST_CAMERA:-false}
source /opt/ros/humble/setup.bash
bash $DIR/stop_sim.sh >/dev/null 2>&1
T0=$(date +%s)
setsid bash $DIR/run_sim.sh ur5e scene:=conveyor continuous:=true gazebo_gui:=false launch_rviz:=false "${LA[@]}" > $OUT/sim.out 2>&1 &
for i in $(seq 1 300); do grep -q "Failed <<<\|Summary:" $OUT/sim.out && break; sleep 0.5; done
grep -q "Failed <<<" $OUT/sim.out && { grep -n "error\|Failed" $OUT/sim.out | head; exit 1; }
source ~/ros2_ws/install/setup.bash
for i in $(seq 1 60); do timeout 3 ros2 topic echo /conveyor/encoder --once >/dev/null 2>&1 && break; sleep 1; done
[ "$RATE" != "1" ] && gz physics -u $(python3 -c "print(int(1000*$RATE))")
T1=$(date +%s)
timeout 900 ros2 run ur_pick_place tracking_sorter.py --cycles $ITEMS --plan-margin $MARGIN > $OUT/sorter.out 2>&1
echo "sorter exit $?, sorting took $(( $(date +%s)-T1 )) s real, whole run $(( $(date +%s)-T0 )) s"
timeout 5 gz stats -d 2 2>/dev/null | tail -1
gz physics -u 1000 >/dev/null 2>&1  # back to real time so grading sees a slow-moving belt
timeout 120 ros2 run ur_pick_place score_bins.py --items $((ITEMS+2)) 2>/dev/null | tail -1
bash $DIR/stop_sim.sh >/dev/null 2>&1
grep -E "ERROR|process has died" $OUT/sorter.out $OUT/sim.out | grep -v hold_joints | cut -c1-170 | head -5
python3 - <<P
import json,os
d=json.load(open(os.path.expanduser('$OUT/tracking_result.json')))
t=[i['t_done'] for i in d['items']]; c=[b-a for a,b in zip(t,t[1:])]
print('label_correct',d['label_correct'],'picked',d['picked'],'failed',len(d['failed']),'missed',d['missed'],'cycle mean',round(d['mean_cycle_s'],2),'min',round(min(c),2),'max',round(d['max_cycle_s'],2),'camera speed',round(d['belt_speed_camera_mean'],4))
print('fail reasons',[f['result'] for f in d['failed']])
P
