#!/usr/bin/env bash
# Soak the Zenoh build against one distribution's router: device reset, then <hours>
# of the earlier soaks' traffic (a counting subscriber; each minute devices, range
# and ping calls, a parameter get and set; a dump every 10 minutes).
#   docker/distros/run_soak.sh <distro> [hours]
D=$1; H=${2:-12}; SECS=$((H * 3600)); C=raftros-$D
OUT=$HOME/distro-tests/$D/soak; mkdir -p $OUT; rm -f $OUT/traffic.jsonl $OUT/counts.jsonl
for other in $(docker ps --format '{{.Names}}' | grep '^raftros-'); do docker stop -t 2 $other > /dev/null; done
docker rm -f $C > /dev/null 2>&1
docker run -d --init --name $C --network host -v $OUT:/out -v $(dirname $(readlink -f $0))/tests:/opt/raftros-tests:ro raftros-ros:$D > /dev/null
docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; exec ros2 run rmw_zenoh_cpp rmw_zenohd >> /out/zenohd.log 2>&1"
until ss -ltn | grep -q ':7447 '; do sleep 0.2; done
curl -s --max-time 5 http://192.168.86.230/api/reset > /dev/null; sleep 5
until curl -s --max-time 2 http://192.168.86.230/api/rosstat | grep -q '"conn":"ready"'; do sleep 1; done; sleep 20
echo "soak $D ${H} h start $(date -Is)" > $OUT/soak.txt
docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; python3 /opt/raftros-tests/counter.py $SECS /out/counts.jsonl"
docker exec $C /opt/raftros-tests/traffic.sh $SECS 60 /out/traffic.jsonl > /dev/null
sleep 90
echo "soak $D end $(date -Is)" >> $OUT/soak.txt
docker exec $C python3 /opt/raftros-tests/summary.py /out >> $OUT/soak.txt
echo "SOAKDONE $D" >> $OUT/soak.txt
