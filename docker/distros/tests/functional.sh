#!/usr/bin/env bash
# RaftROS functional checks against the ExampleDiscoverable firmware, run inside a
# raftros-ros:<distro> container with a router already up.  One PASS/FAIL line per
# check.  Usage: functional.sh [zenoh|fastdds|cyclone]   (default zenoh)
#
# ros2 CLI processes ignore SIGTERM, so every call uses `timeout -s KILL`.
# ROS's setup.bash reads unset variables, so source it before `set -u`
source /opt/ros/${ROS_DISTRO}/setup.bash
set -u
case "${1:-zenoh}" in
    zenoh)   export RMW_IMPLEMENTATION=rmw_zenoh_cpp ;;
    fastdds) export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ;;
    cyclone) export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ;;
esac
MODE=${1:-zenoh}
NODE=/raft_esp32
RANGE=/raft/range_1_29
RANGE_HASH=RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a
PASS=0; FAIL=0
T() { timeout -s KILL "$@"; }
check() {   # check <name> <0|1> <detail>
    if [ "$2" = 0 ]; then PASS=$((PASS+1)); echo "PASS  $1  $3"; else FAIL=$((FAIL+1)); echo "FAIL  $1  $3"; fi
}
has() { grep -q -- "$1" <<< "$2" && echo 0 || echo 1; }
retry() {   # retry <tries> <cmd...> : graph queries from a fresh process can race discovery
    local n=$1; shift; local out=""
    for ((i=0; i<n; i++)); do out=$("$@" 2>&1) && [ -n "$out" ] && { echo "$out"; return; }; sleep 2; done; echo "$out"
}

echo "== $ROS_DISTRO / $RMW_IMPLEMENTATION / $(date -Is)"
[ "$MODE" != zenoh ] && ros2 daemon stop > /dev/null 2>&1

out=$(retry 5 T 20 ros2 node list --no-daemon); check "node list" "$(has "$NODE" "$out")" "$(tr '\n' ' ' <<< "$out")"
out=$(retry 5 T 20 ros2 node info --no-daemon $NODE)
check "node info publishers" "$( [ "$(has /chatter: "$out")$(has "$RANGE:" "$out")" = 00 ] && echo 0 || echo 1)" "/chatter + $RANGE"
check "node info subscribers" "$( [ "$(has /chatter_in: "$out")$(has /chatter_in2: "$out")" = 00 ] && echo 0 || echo 1)" "/chatter_in + /chatter_in2"
if [ "$MODE" = zenoh ]; then
    nsvc=$(sed -n '/Service Servers:/,/Service Clients:/p' <<< "$out" | grep -c "/raft_esp32/")
    check "node info services" "$( [ "$nsvc" -ge 10 ] && echo 0 || echo 1)" "$nsvc service servers"
fi
out=$(retry 5 T 20 ros2 topic info -v --no-daemon $RANGE)
check "topic info node" "$(has "Node name: raft_esp32" "$out")" ""
check "topic info type hash" "$(has "$RANGE_HASH" "$out")" "$(grep -o 'RIHS01_[0-9a-f]*' <<< "$out" | head -1)"
check "topic info qos" "$( [ "$(has BEST_EFFORT "$out")$(has VOLATILE "$out")" = 00 ] && echo 0 || echo 1)" "BEST_EFFORT/VOLATILE"
out=$(T 20 ros2 topic echo --once --qos-reliability best_effort $RANGE sensor_msgs/msg/Range 2>&1)
check "echo range" "$(has "range:" "$out")" "$(grep -m1 '^range:' <<< "$out")"
out=$(T 20 ros2 topic echo --once /chatter std_msgs/msg/String 2>&1)
check "echo chatter" "$(has "Hello from raft_esp32" "$out")" "$(grep -m1 'data:' <<< "$out")"

if [ "$MODE" = zenoh ]; then
    out=$(T 25 ros2 service call /raft_esp32/devices std_srvs/srv/Trigger 2>&1)
    before=$(grep -o 'chatter_in rx [0-9]*' <<< "$out" | grep -o '[0-9]*$')
    T 15 ros2 topic pub --once -w 0 /chatter_in std_msgs/msg/String "{data: 'distro test $ROS_DISTRO'}" > /dev/null 2>&1
    sleep 1
    out=$(T 25 ros2 service call /raft_esp32/devices std_srvs/srv/Trigger 2>&1)
    after=$(grep -o 'chatter_in rx [0-9]*' <<< "$out" | grep -o '[0-9]*$')
    check "inbound /chatter_in" "$( [ -n "$before" ] && [ -n "$after" ] && [ "$after" -gt "$before" ] && echo 0 || echo 1)" "rx $before -> $after"
    out=$(T 25 ros2 service call /raft_esp32/ping std_srvs/srv/Empty 2>&1)
    check "service ping (Empty)" "$(has "Empty_Response" "$out")" ""
    out=$(T 25 ros2 service call /raft_esp32/range std_srvs/srv/Trigger 2>&1)
    check "service range (deferred Trigger)" "$(has "success=True" "$out")" "$(grep -o "message='[^']*'" <<< "$out")"
    out=$(T 25 ros2 service call /raft_esp32/chatter_enable std_srvs/srv/SetBool "{data: true}" 2>&1)
    check "service chatter_enable (SetBool)" "$(has "success=True" "$out")" ""
    out=$(T 25 ros2 param list $NODE 2>&1)
    check "param list" "$( [ "$(has chatterPeriodMs "$out")$(has routerHost "$out")$(has use_sim_time "$out")" = 000 ] && echo 0 || echo 1)" "$(tr -s ' \n' ' ' <<< "$out")"
    out=$(T 25 ros2 param get $NODE chatterPeriodMs 2>&1)
    check "param get" "$(has "Integer value is" "$out")" "$out"
    out=$(T 25 ros2 param set $NODE chatterPeriodMs 5 2>&1)
    check "param set refused" "$(has "100-60000" "$out")" "$out"
    out=$(T 25 ros2 param set $NODE chatterPeriodMs 1000 2>&1)
    check "param set accepted" "$(has "successful" "$out")" "$out"
    out=$(T 25 ros2 param describe $NODE chatterPeriodMs 2>&1)
    check "param describe" "$(has "Type: integer" "$out")" "$(grep -m1 Description <<< "$out")"
    out=$(T 30 ros2 param dump $NODE 2>&1)
    check "param dump" "$(has "ros__parameters" "$out")" ""
else
    T 15 ros2 topic pub --once -w 1 /chatter_in std_msgs/msg/String "{data: 'distro test $ROS_DISTRO'}" > /dev/null 2>&1
    check "inbound /chatter_in (sent)" "$?" "(device counter checked by the host script)"
fi
echo "RESULT $ROS_DISTRO $MODE: $PASS passed, $FAIL failed"
