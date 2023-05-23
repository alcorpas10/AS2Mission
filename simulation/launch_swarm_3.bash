#!/bin/bash

drone_namespaces=('drone_sim_0' 'drone_sim_1' 'drone_sim_2')
simulation_config="simulation_config/swarm3.json"

# For each drone namespace
for drone_namespace in "${drone_namespaces[@]}"
do
    ./as2_launch.bash "${drone_namespace}" $simulation_config
done

# Attach to the first session
session=${drone_namespaces[0]}
# if inside a tmux session detach before attaching to the session
if [ -n "$TMUX" ]; then
    tmux switch-client -t $session
else
    tmux attach -t $session:0
fi

