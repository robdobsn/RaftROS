#!/usr/bin/env bash
# Service and parameter traffic against the device, as in the earlier soaks:
# each round a devices, read (deferred) and ping call, a parameter get and set,
# and every 10th round a full dump; device rosstat recorded each round.
# Usage: traffic.sh <seconds> <interval_s> <out.jsonl>
#   RAFTROS_NODE=/<node> (default: the first node in the graph)
#   RAFTROS_DEVICE_IP=<ip> for /api/rosstat (default 192.168.86.230)
source /opt/ros/${ROS_DISTRO}/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp
NODE=${RAFTROS_NODE:-$(timeout -s KILL 20 ros2 node list --no-daemon | grep -v -e foxglove -e _ros2cli -e '^$' | head -1)}
DEV=${RAFTROS_DEVICE_IP:-192.168.86.230}
DUR=$1; GAP=$2; OUT=$3; END=$(( $(date +%s) + DUR )); i=0
ok() { timeout -s KILL 20 "$@" > /tmp/traffic_last.txt 2>&1; grep -qE "success=True|Empty_Response|Set parameter successful|value is:|ros__parameters" /tmp/traffic_last.txt && echo 1 || echo 0; }
while [ $(date +%s) -lt $END ]; do
    t0=$(date +%s); i=$((i+1)); p=$(( i % 2 ? 900 : 1000 ))
    dev=$(ok ros2 service call $NODE/devices std_srvs/srv/Trigger)
    rd=$(ok ros2 service call $NODE/read std_srvs/srv/Trigger)
    png=$(ok ros2 service call $NODE/ping std_srvs/srv/Empty)
    pg=$(ok ros2 param get $NODE chatterEnable)
    ps=$(ok ros2 param set $NODE chatterPeriodMs $p)
    dmp=-; [ $(( i % 10 )) -eq 0 ] && dmp=$(ok ros2 param dump $NODE)
    st=$(curl -s -m 5 http://$DEV/api/rosstat)
    echo "{\"t\":$t0,\"i\":$i,\"dev\":$dev,\"read\":$rd,\"ping\":$png,\"pget\":$pg,\"pset\":$ps,\"dump\":\"$dmp\",\"stat\":${st:-null}}" >> $OUT
    rest=$(( GAP - ( $(date +%s) - t0 ) )); [ $rest -gt 0 ] && sleep $rest
done
echo done
