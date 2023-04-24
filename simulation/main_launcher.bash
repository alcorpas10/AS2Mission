#!/bin/bash

$drone_namespace=$1
if [ -z "$drone_namespace" ]; then
    drone_namespace="drone_sim_0"
fi
simulation_config="simulation_config/default.json"
launch_keyboard_teleop="true"

# Run nodes
./as2_launch.bash "${drone_namespace}" $simulation_config $launch_keyboard_teleop

session=${drone_namespace}

# if inside a tmux session detach before attaching to the session
if [ -n "$TMUX" ]; then
    tmux switch-client -t $session
else
    tmux attach -t $session:0
fi