#!/usr/bin/env bash
# RaftROS RTPS-build checks against one ROS 2 distribution's DDS implementations.
#   docker/distros/run_rtps.sh <jazzy|kilted|lyrical>
# The board must be running the RTPS firmware (CONFIG_RAFTROS_BACKEND_RTPS=y).
# No router: discovery is DDS multicast on the host's LAN (container uses host networking).
set -u
D=$1; DEV=192.168.86.230
OUT=$HOME/distro-tests/$D; mkdir -p $OUT; LOG=$OUT/rtps.log; : > $LOG
C=raftros-$D
for other in $(docker ps --format '{{.Names}}' | grep '^raftros-'); do docker stop -t 2 $other > /dev/null; done
docker rm -f $C > /dev/null 2>&1
docker run -d --init --name $C --network host -v $OUT:/out -v $(dirname $(readlink -f $0))/tests:/opt/raftros-tests:ro raftros-ros:$D > /dev/null
echo "== rtps $D $(date +%T)" | tee -a $LOG
curl -s --max-time 5 http://$DEV/api/rosstat | tee -a $LOG; echo | tee -a $LOG
for mode in fastdds cyclone; do
    docker exec $C /opt/raftros-tests/functional.sh $mode 2>&1 | tee $OUT/rtps-$mode.txt | grep -E "PASS|FAIL|RESULT" | tee -a $LOG
done
curl -s --max-time 5 http://$DEV/api/rosstat | tee -a $LOG; echo | tee -a $LOG
echo "== rtps $D done $(date +%T)" | tee -a $LOG
