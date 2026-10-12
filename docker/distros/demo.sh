#!/usr/bin/env bash
# Demo / photo setup on the ROS host: a Zenoh router and foxglove_bridge for one
# distribution, so Foxglove (desktop app or app.foxglove.dev) on any laptop can
# connect to ws://<this host>:8765.  Optionally the DemoSimple terminal dashboard.
#   docker/distros/demo.sh <distro>              router + foxglove_bridge (background)
#   docker/distros/demo.sh <distro> dashboard    ... and run DemoSimple here (Ctrl+C to quit)
#   docker/distros/demo.sh <distro> stop         stop the bridge
# With RAFTROS_SERIAL=<port> the router also listens on that serial port for a
# board using the Zenoh serial link (RaftROS "link":"usbjtag"), e.g.
#   RAFTROS_SERIAL=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_70:04:1D:AD:BF:B4-if00 demo.sh lyrical
# The container then gets the host's /dev and access to USB serial devices, so
# the port survives the board re-enumerating when it resets.  Nothing else may
# hold the port (raft monitor, a serial terminal) while the router has it.
set -u
D=$1; MODE=${2:-}; C=raftros-$D; HERE=$(dirname $(readlink -f $0))
SERIAL=${RAFTROS_SERIAL:-}; BAUD=${RAFTROS_SERIAL_BAUD:-921600}
if pgrep -f "[r]un_soak.sh" > /dev/null; then echo "A soak is running - not touching the routers."; exit 1; fi
if [ "$MODE" = stop ]; then docker exec $C pkill -f foxglove_bridge; exit 0; fi
for other in $(docker ps --format '{{.Names}}' | grep '^raftros-' | grep -v "^$C$"); do docker stop -t 2 $other > /dev/null; done
# A running container without device access cannot reach a serial port: replace it
if [ -n "$SERIAL" ] && docker ps --format '{{.Names}}' | grep -q "^$C$" && \
   ! docker inspect $C --format '{{.HostConfig.DeviceCgroupRules}}' | grep -q '166'; then
    echo "Recreating $C with access to serial devices"; docker rm -f $C > /dev/null
fi
if ! docker ps --format '{{.Names}}' | grep -q "^$C$"; then
    docker rm -f $C > /dev/null 2>&1
    DEVICES=()      # ttyACM (CDC, USB-Serial-JTAG) is major 166, ttyUSB 188
    [ -n "$SERIAL" ] && DEVICES=(-v /dev:/dev "--device-cgroup-rule=c 166:* rmw" "--device-cgroup-rule=c 188:* rmw")
    docker run -d --init --name $C --network host -v $HOME/distro-tests/$D:/out "${DEVICES[@]}" \
        -v $HERE/tests:/opt/raftros-tests:ro -v $HERE/demo:/opt/raftros-demo:ro raftros-ros:$D > /dev/null
fi
ROUTER_ENV=""
if [ -n "$SERIAL" ]; then
    # The default router config with the serial endpoint added beside TCP 7447,
    # which the ros2 CLI and foxglove_bridge still use
    docker exec $C bash -c "sed 's#\"tcp/\[::\]:7447\"#\"tcp/[::]:7447\", \"serial/$SERIAL\#baudrate=$BAUD\"#' \
        /opt/ros/$D/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5 > /out/router_serial.json5"
    docker exec $C grep -q "serial/$SERIAL" /out/router_serial.json5 || { echo "Could not add $SERIAL to the router config"; exit 1; }
    ROUTER_ENV="export ZENOH_ROUTER_CONFIG_URI=/out/router_serial.json5;"
    if ss -ltn | grep -q ':7447 ' && ! docker exec $C pgrep -f rmw_zenohd > /dev/null; then
        echo "Something other than this container's router holds :7447 - stop it first"; exit 1
    fi
    if docker exec $C pgrep -f rmw_zenohd > /dev/null && [ "$(docker exec $C cat /out/router_mode 2>/dev/null)" != "serial:$SERIAL" ]; then
        echo "Restarting the router with the serial endpoint"; docker exec $C pkill -f rmw_zenohd
        while ss -ltn | grep -q ':7447 '; do sleep 0.2; done
    fi
fi
if ! ss -ltn | grep -q ':7447 '; then
    docker exec $C bash -c "echo ${SERIAL:+serial:$SERIAL} > /out/router_mode"
    docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; $ROUTER_ENV exec ros2 run rmw_zenoh_cpp rmw_zenohd >> /out/zenohd.log 2>&1"
fi
until ss -ltn | grep -q ':7447 '; do sleep 0.2; done
ss -ltn | grep -q ':8765 ' || docker exec -d $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml >> /out/foxglove.log 2>&1"
until ss -ltn | grep -q ':8765 '; do sleep 0.5; done
[ -n "$SERIAL" ] && echo "Router also listening on serial/$SERIAL at $BAUD baud"
echo "Router and foxglove_bridge ($D) running.  In Foxglove: Open connection -> Foxglove WebSocket -> ws://$(hostname -I | awk '{print $1}'):8765"
if [ "$MODE" = dashboard ]; then
    docker exec -it $C bash -c "source /opt/ros/$D/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp; python3 -u /opt/raftros-demo/raftros_dynamic_dashboard.py"
fi
