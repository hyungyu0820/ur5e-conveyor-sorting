#!/bin/bash
# One headless exception-handling run on the belt that never stops: start the cell with faults,
# feed N items, let one sorter work, record the simulator's truth, shut down.
#   bash tools/exception_trial.sh <baseline|recovery|overlap> <seed> [--rate R] [--items N] [--tag T] [sorter args...]
# baseline = tracking_sorter.py as it is (a failed grasp is given up, the item rides on)
# recovery = recovery_sorter.py --overlap pass (wrist-camera check, retry, turned re-grasp, hold bin)
# overlap  = recovery_sorter.py as it is (default --overlap separate: two items together are taken apart too)
# NOTE: the output folder is emptied first. Use --tag to keep an earlier run with the same mode, seed and rate.
# --rate: simulation speed target (1 = real time). Final numbers: --rate 1.
# Output: ~/ros2_ws/log/sim/exception/<mode>_seed<seed>_rate<R>[_tag]/ {robot.json, truth.json, sim.out, sorter.out}
MODE=$1; SEED=$2; shift 2
RATE=1; ITEMS=30; TAG=""; SA=()
while [ $# -gt 0 ]; do case "$1" in
  --rate) RATE=$2; shift 2;; --items) ITEMS=$2; shift 2;; --tag) TAG=_$2; shift 2;; *) SA+=("$1"); shift;; esac; done
DIR=~/ros2_ws/src/ur_pick_place; OUT=~/ros2_ws/log/sim/exception/${MODE}_seed${SEED}_rate${RATE}${TAG}
rm -rf $OUT; mkdir -p $OUT
case "$MODE" in
  baseline) SORTER=tracking_sorter.py; export UR_WRIST_CAMERA=${UR_WRIST_CAMERA:-false};;
  recovery) SORTER=recovery_sorter.py; SA+=(--overlap pass); export UR_WRIST_CAMERA=true; export UR_WRIST_CAMERA_MOUNT=raised_wide;;
  overlap) SORTER=recovery_sorter.py; export UR_WRIST_CAMERA=true; export UR_WRIST_CAMERA_MOUNT=raised_wide;;
  *) echo "mode must be baseline, recovery or overlap"; exit 2;; esac
source /opt/ros/humble/setup.bash
bash $DIR/stop_sim.sh >/dev/null 2>&1
T0=$(date +%s)
setsid bash $DIR/run_sim.sh ur5e scene:=conveyor continuous:=true gazebo_gui:=false launch_rviz:=false \
  belt_speed:=0.10 item_spacing:=0.5 faults:=true seed:=$SEED feed_limit:=$ITEMS wait_start:=true > $OUT/sim.out 2>&1 &
for i in $(seq 1 300); do grep -q "Failed <<<\|Summary:" $OUT/sim.out && break; sleep 0.5; done
grep -q "Failed <<<" $OUT/sim.out && { grep -n "error\|Failed" $OUT/sim.out | head; exit 1; }
source ~/ros2_ws/install/setup.bash
for i in $(seq 1 60); do timeout 3 ros2 topic echo /conveyor/encoder --once >/dev/null 2>&1 && break; sleep 1; done
[ "$RATE" != "1" ] && gz physics -u $(python3 -c "print(int(1000*$RATE))")
T1=$(date +%s)
LIB=~/ros2_ws/install/ur_pick_place/lib/ur_pick_place
python3 $LIB/exception_metrics.py --out $OUT/truth.json > $OUT/metrics.out 2>&1 &
REC=$!
EXC_RUN_DIR=$OUT timeout 900 python3 $LIB/$SORTER --until-fed --out $OUT/robot.json "${SA[@]}" > $OUT/sorter.out 2>&1
echo "sorter exit $?, sorting took $(( $(date +%s)-T1 )) s real, whole run $(( $(date +%s)-T0 )) s"
timeout 5 gz stats -d 2 2>/dev/null | tail -1 | tee $OUT/gzstats.out
sleep 3   # the last item settles and the truth topic catches up
kill -TERM $REC; wait $REC 2>/dev/null
tail -1 $OUT/metrics.out
bash $DIR/stop_sim.sh >/dev/null 2>&1
grep -E "ERROR|process has died|Traceback" $OUT/sorter.out $OUT/sim.out | grep -v hold_joints | cut -c1-200 | head -5
echo "output: $OUT"
