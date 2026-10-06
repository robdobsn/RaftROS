#!/usr/bin/env bash
# Demo / photo setup on the ROS host: a Zenoh router and foxglove_bridge for one
# distribution, so Foxglove (desktop app or app.foxglove.dev) on any laptop can
# connect to ws://<this host>:8765.  Optionally the DemoSimple terminal dashboard.
#   docker/distros/demo.sh <distro>              router + foxglove_bridge (background)
#   docker/distros/demo.sh <distro> dashboard    ... and run DemoSimple here (Ctrl+C to quit)
#   docker/distros/demo.sh <distro> stop         stop the bridge
set -u
D=$1; MODE=${2:-}; C=raftros-$D; HERE=$(dirname $(readlink -f $0))
if pgrep -f "[r]un_soak.sh" > /dev/null; then echo "A soak is running - not touching the routers."; exit 1; fi
if [ "$MODE" = stop ]; then docker exec $C pkill -f foxglove_bridge; exit 0; fi
for other in $(docker ps --format '{{.Names}}' | grep '^raftros-' | grep -v "^$C$"); do docker stop -t 2 $other > /dev/null; done
if ! docker ps --format '{{.Names}}' | grep -q "^$C$"; then
    docker rm -f $C > /dev/null 2>&1
    docker run -d --init --name $C --network host -v $HOME/distro-tests/$D:/out \
        -v $HERE/tests:/opt/raftros-tests:ro -v $HERE/demo:/opt/raftros-demo:ro raftros-ros:$D > /dev/null
fi
ss -ltn | grep -q ':7447 ' || docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; exec ros2 run rmw_zenoh_cpp rmw_zenohd >> /out/zenohd.log 2>&1"
until ss -ltn | grep -q ':7447 '; do sleep 0.2; done
ss -ltn | grep -q ':8765 ' || docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml >> /out/foxglove.log 2>&1"
until ss -ltn | grep -q ':8765 '; do sleep 0.5; done
echo "Router and foxglove_bridge ($D) running.  In Foxglove: Open connection -> Foxglove WebSocket -> ws://$(hostname -I | awk '{print $1}'):8765"
if [ "$MODE" = dashboard ]; then
    docker exec -it $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; python3 -u /opt/raftros-demo/raftros_dynamic_dashboard.py"
fi
