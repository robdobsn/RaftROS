#!/usr/bin/env bash
# Start one distribution's router and watch hot-plug for <seconds>.
#   docker/distros/run_hotplug.sh <distro> [seconds]
D=$1; SECS=${2:-180}; C=raftros-$D; OUT=$HOME/distro-tests/$D; mkdir -p $OUT
for other in $(docker ps --format '{{.Names}}' | grep '^raftros-' | grep -v "^$C$"); do docker stop -t 2 $other > /dev/null; done
if ! docker ps --format '{{.Names}}' | grep -q "^$C$"; then
    docker rm -f $C > /dev/null 2>&1
    docker run -d --init --name $C --network host -v $OUT:/out -v $(dirname $(readlink -f $0))/tests:/opt/raftros-tests:ro raftros-ros:$D > /dev/null
fi
ss -ltn | grep -q ':7447 ' || docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; exec ros2 run rmw_zenoh_cpp rmw_zenohd >> /out/zenohd.log 2>&1"
until ss -ltn | grep -q ':7447 '; do sleep 0.2; done
until curl -s --max-time 2 http://192.168.86.230/api/rosstat | grep -q '"conn":"ready"'; do sleep 0.5; done
sleep 5
echo "== hot-plug watch $D for ${SECS} s - unplug the sensor, wait ~30 s, plug it back" | tee $OUT/hotplug.log
docker exec $C /opt/zenoh-venv/bin/python /opt/raftros-tests/hotplug_watch.py $SECS | tee -a $OUT/hotplug.log
