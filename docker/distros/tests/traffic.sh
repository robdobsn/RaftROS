#!/usr/bin/env bash
# Service and parameter traffic against the device, as in the earlier soaks:
# each round a devices, range (deferred) and ping call, a parameter get and set,
# and every 10th round a full dump; device rosstat recorded each round.
# Usage: traffic.sh <seconds> <interval_s> <out.jsonl>
source /opt/ros/${ROS_DISTRO}/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp
DUR=$1; GAP=$2; OUT=$3; END=$(( $(date +%s) + DUR )); i=0
ok() { timeout -s KILL 20 "$@" > /tmp/traffic_last.txt 2>&1; grep -qE "success=True|Empty_Response|Set parameter successful|value is:|ros__parameters" /tmp/traffic_last.txt && echo 1 || echo 0; }
while [ $(date +%s) -lt $END ]; do
    t0=$(date +%s); i=$((i+1)); p=$(( i % 2 ? 900 : 1000 ))
    dev=$(ok ros2 service call /raft_esp32/devices std_srvs/srv/Trigger)
    rng=$(ok ros2 service call /raft_esp32/range std_srvs/srv/Trigger)
    png=$(ok ros2 service call /raft_esp32/ping std_srvs/srv/Empty)
    pg=$(ok ros2 param get /raft_esp32 chatterEnable)
    ps=$(ok ros2 param set /raft_esp32 chatterPeriodMs $p)
    dmp=-; [ $(( i % 10 )) -eq 0 ] && dmp=$(ok ros2 param dump /raft_esp32)
    st=$(curl -s -m 5 http://192.168.86.230/api/rosstat)
    echo "{\"t\":$t0,\"i\":$i,\"dev\":$dev,\"rng\":$rng,\"ping\":$png,\"pget\":$pg,\"pset\":$ps,\"dump\":\"$dmp\",\"stat\":${st:-null}}" >> $OUT
    rest=$(( GAP - ( $(date +%s) - t0 ) )); [ $rest -gt 0 ] && sleep $rest
done
echo done
