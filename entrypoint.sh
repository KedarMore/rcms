#!/bin/bash

# Function to handle the stop signal
cleanup() {
    if [ -n "$node_pid" ]; then
        kill -2 $node_pid
    fi
    while ps -p $node_pid > /dev/null; do
        sleep 0.1
    done
    exit 0
}

trap cleanup SIGTERM

source ./install/setup.bash
ros2 launch rcs_config_manager rcs_config_manager_launch.py $@ &
pid=$!
sleep 1
node_pid=$(pgrep --parent "$pid")
wait "$pid"
exit 1
