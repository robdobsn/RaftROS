#!/usr/bin/env bash
# Run the RaftROS Zenoh test suite against one ROS 2 distribution, on the ROS host.
#   docker/distros/run_suite.sh <jazzy|kilted|lyrical> [load_seconds]
# Needs: the raftros-ros:<distro> image, the board running the Zenoh firmware with
# routerHost = this host, tshark/dumpcap usable without sudo.  Results go to
# ~/distro-tests/<distro>/.  Only one distribution's router may run at a time.
set -u
D=$1; LOAD=${2:-600}
DEV=192.168.86.230
OUT=$HOME/distro-tests/$D; mkdir -p $OUT; LOG=$OUT/suite.log; : > $LOG
C=raftros-$D
say() { echo "$(date +%T) $*" | tee -a $LOG; }
stat() { echo "-- rosstat: $1" >> $LOG; curl -s --max-time 5 http://$DEV/api/rosstat >> $LOG; echo >> $LOG; }
ready_wait() { local t0=$(date +%s.%N); until curl -s --max-time 2 http://$DEV/api/rosstat | grep -q '"conn":"ready"'; do sleep 0.2; done; echo "$(echo "$(date +%s.%N) - $t0" | bc | cut -c1-5)"; }
x() { docker exec $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; $*"; }
router_start() { docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; exec ros2 run rmw_zenoh_cpp rmw_zenohd >> /out/zenohd.log 2>&1"; until ss -ltn | grep -q ':7447 '; do sleep 0.2; done; }
router_stop() { docker exec $C pkill -f rmw_zenohd; while ss -ltn | grep -q ':7447 '; do sleep 0.2; done; }

say "== suite $D, image $(docker image inspect raftros-ros:$D --format '{{.Id}}' | cut -c8-19)"
# Exactly one router on this host: stop other distributions' containers
for other in $(docker ps --format '{{.Names}}' | grep '^raftros-' | grep -v "^$C$"); do docker stop -t 2 $other > /dev/null; done
docker rm -f $C > /dev/null 2>&1
# Test scripts are mounted from the host, so fixes need no image rebuild
docker run -d --init --name $C --network host -v $OUT:/out -v $(dirname $(readlink -f $0))/tests:/opt/raftros-tests:ro raftros-ros:$D > /dev/null
x "ros2 pkg list | grep -E '^rmw_zenoh_cpp$'; dpkg -s ros-$D-rmw-zenoh-cpp | grep ^Version" | tee -a $LOG

# Session bring-up, captured from a device reset
dumpcap -q -i any -f "tcp port 7447" -a duration:45 -w $OUT/session.pcapng > /dev/null 2>&1 &
CAP=$!; sleep 2
router_start
curl -s --max-time 5 http://$DEV/api/reset > /dev/null
sleep 5; say "device ready $(ready_wait) s after reset+5 s"
wait $CAP; chmod 644 $OUT/session.pcapng
# Ubuntu 26.04's AppArmor profile lets tshark read /tmp but not this folder
cp $OUT/session.pcapng /tmp/raftros-session-$D.pcapng
say "capture: $(capinfos -c -M $OUT/session.pcapng | sed -n 's/.*packets: *//p') packets in session.pcapng"
tshark -r /tmp/raftros-session-$D.pcapng -Y "tcp.port == 7447 && tcp.len > 0" 2>/dev/null | head -12 >> $LOG
sleep 10; stat "after bring-up"

say "-- functional"; x "/opt/raftros-tests/functional.sh zenoh" > $OUT/functional.txt 2>&1; tail -1 $OUT/functional.txt | tee -a $LOG
grep FAIL $OUT/functional.txt | tee -a $LOG

say "-- raw queries"; docker exec $C /opt/zenoh-venv/bin/python /opt/raftros-tests/raw_queries.py 2>&1 | tee -a $LOG; stat "raw queries"
say "-- 3000-byte string to /chatter_in"; x "timeout -s KILL 15 ros2 topic pub --once -w 0 /chatter_in std_msgs/msg/String \"{data: '\$(printf 'x%.0s' \$(seq 3000))'}\" > /dev/null 2>&1; echo exit \$?" | tee -a $LOG; stat "3 kB string"
say "-- 8 concurrent range calls"
x "for i in 1 2 3 4 5 6 7 8; do (timeout -s KILL 20 ros2 service call /raft_esp32/range std_srvs/srv/Trigger 2>&1 | grep -o 'success=True\|z_reply_is_ok returned False' | head -1 | sed \"s/^/call \$i: /\") & done; wait" | sort | tee -a $LOG; stat "burst"
x "timeout -s KILL 20 ros2 service call /raft_esp32/ping std_srvs/srv/Empty 2>&1 | grep -o Empty_Response" | sed 's/^/after burst: /' | tee -a $LOG

say "-- graph visibility (40 liveliness queries)"; docker exec $C /opt/zenoh-venv/bin/python /opt/raftros-tests/liveliness.py 40 | tee -a $LOG

for n in 1 2; do
    say "-- router restart $n (15 s down)"; router_stop
    until curl -s --max-time 2 http://$DEV/api/rosstat | grep -q '"conn":"disconnected"'; do sleep 0.5; done
    sleep 15; router_start; say "   session ready $(ready_wait) s after the router was listening"
done
sleep 5; stat "after router restarts"

say "-- routerHost set through the parameter service"
x "timeout -s KILL 20 ros2 param set /raft_esp32 routerHost $(hostname -I | awk '{print $1}')" | tee -a $LOG
sleep 1; say "   ready again after $(ready_wait) s"; stat "after routerHost set"

say "-- boot with no router (60 s)"; router_stop
curl -s --max-time 5 http://$DEV/api/reset > /dev/null; sleep 60; stat "60 s after reset, no router"
router_start; say "   ready $(ready_wait) s after the router started"; sleep 10

say "-- functional again"; x "/opt/raftros-tests/functional.sh zenoh" > $OUT/functional2.txt 2>&1; tail -1 $OUT/functional2.txt | tee -a $LOG

say "-- load ${LOAD} s"; rm -f $OUT/traffic.jsonl $OUT/counts.jsonl; stat "before load"
docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; python3 /opt/raftros-tests/counter.py $LOAD /out/counts.jsonl"
x "/opt/raftros-tests/traffic.sh $LOAD 5 /out/traffic.jsonl" > /dev/null
sleep 5; stat "after load"
docker exec $C python3 /opt/raftros-tests/summary.py /out | tee -a $LOG
say "== suite $D done"
