#!/bin/bash
# Record a demo clip of an exception run with the overview camera (NOT a measurement: the simulation
# runs slowed down so that software rendering keeps up; frames are stamped with simulation time).
#   bash tools/record_clip.sh <recovery|overlap> <seed> <items> [slow factor, default 0.1] [camera fps, default 15]
# The picture is 448 x 336 on purpose. Larger images lost frames on the way to the recorder however slowly
# the simulation ran: in one test run a 448 x 336 camera delivered 15.0 of 15 frames per second and a
# 560 x 420 camera beside it 5.9, with gaps up to 0.47 s (image messages above about 0.5 MB get dropped).
# Output: ~/ros2_ws/log/sim/clips/<mode>_seed<seed>/ {frames/*.jpg, robot.json, sorter.out}
# Then cut a GIF: python3 tools/record_clip.py gif <that folder>/frames out.gif --from T0 --to T1
MODE=$1; SEED=$2; ITEMS=$3; SLOW=${4:-0.1}; FPS=${5:-15}
DIR=~/ros2_ws/src/ur_pick_place; OUT=~/ros2_ws/log/sim/clips/${MODE}_seed${SEED}
rm -rf $OUT; mkdir -p $OUT/frames
SA=(); [ "$MODE" = "recovery" ] && SA+=(--overlap pass)
export UR_WRIST_CAMERA=true; export UR_WRIST_CAMERA_MOUNT=raised_wide
# Recording only: let image messages of any size go through shared memory. With the default settings the
# wrist camera's 0.9 MB images mostly got lost once the simulation was slowed down (1.6 frames per simulated
# second), and the robot could not see what it had failed to grasp. The measured runs do NOT use this.
cat > $OUT/dds_big_shm.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor><transport_id>shm_big</transport_id><type>SHM</type><segment_size>33554432</segment_size></transport_descriptor>
    <transport_descriptor><transport_id>udp</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
  <participant profile_name="big_shm" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>shm_big</transport_id><transport_id>udp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
export FASTRTPS_DEFAULT_PROFILES_FILE=$OUT/dds_big_shm.xml
source /opt/ros/humble/setup.bash
bash $DIR/stop_sim.sh >/dev/null 2>&1
setsid bash $DIR/run_sim.sh ur5e scene:=conveyor continuous:=true gazebo_gui:=false launch_rviz:=false \
  belt_speed:=0.10 item_spacing:=0.5 faults:=true seed:=$SEED feed_limit:=$ITEMS wait_start:=true > $OUT/sim.out 2>&1 &
for i in $(seq 1 300); do grep -q "Failed <<<\|Summary:" $OUT/sim.out && break; sleep 0.5; done
grep -q "Failed <<<" $OUT/sim.out && { grep -n "error\|Failed" $OUT/sim.out | head; exit 1; }
source ~/ros2_ws/install/setup.bash
for i in $(seq 1 60); do timeout 3 ros2 topic echo /conveyor/encoder --once >/dev/null 2>&1 && break; sleep 1; done
# the overview camera at the wanted frame rate (the model file itself says 5 frames per second)
sed -e "s#<update_rate>5.0</update_rate>#<update_rate>$FPS</update_rate>#" \
  -e "s#<width>800</width><height>600</height>#<width>448</width><height>336</height>#" \
  $(ros2 pkg prefix ur_pick_place)/share/ur_pick_place/models/overview_camera.sdf > $OUT/overview_camera.sdf
ros2 run gazebo_ros spawn_entity.py -entity overview_camera -file $OUT/overview_camera.sdf > $OUT/spawn.out 2>&1
gz physics -u $(python3 -c "print(int(1000*$SLOW))")
python3 $DIR/tools/record_clip.py record $OUT/frames > $OUT/record.out 2>&1 &
REC=$!
sleep 3
LIB=~/ros2_ws/install/ur_pick_place/lib/ur_pick_place
EXC_RUN_DIR=$OUT timeout 2400 python3 $LIB/recovery_sorter.py --until-fed --out $OUT/robot.json "${SA[@]}" > $OUT/sorter.out 2>&1
echo "sorter exit $?"
timeout 5 gz stats -d 2 2>/dev/null | tail -1
kill -TERM $REC; wait $REC 2>/dev/null; cat $OUT/record.out
bash $DIR/stop_sim.sh >/dev/null 2>&1
echo "output: $OUT"
