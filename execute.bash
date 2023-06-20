#!/bin/bash

SESSION="Mision"
n_drones=$1
window=1

re='^[0-9]+$'

if ! [[ ${n_drones} =~ $re ]] ; then
    echo "usage: ./execute.bash n_drones"; exit 1
fi

# echo ${n_drones}

tmux -2 new-session -d -s $SESSION

for ((i=0; i<${n_drones}; i++)); do
    tmux new-window -t $SESSION:${wiwndow} -n "Monitor ${i}"
    tmux send-keys "ros2 launch monitor execution_monitor.py drone_id:=${i} config_file:=config/config_prueba_swarm_${n_drones}.yaml" C-m
    window=$((${window} + 1))
done

tmux new-window -t $SESSION:${wiwndow} -n 'Planner'
tmux send-keys "ros2 launch planner planner.py config_file:=config/config_prueba_swarm_${n_drones}.yaml" C-m
window=$((${window} + 1))

tmux new-window -t $SESSION:${wiwndow} -n 'Replanner'
tmux send-keys "ros2 launch replanning_manager replanning_manager.py" C-m
window=$((${window} + 1))

tmux new-window -t $SESSION:${wiwndow} -n 'Viewer'
tmux send-keys "ros2 launch viewer viewer.py config_file:=config/config_prueba_swarm_${n_drones}.yaml" C-m
window=$((${window} + 1))

tmux new-window -t $SESSION:${wiwndow} -n 'Execution Manager'
tmux send-keys "python3 code/mission_transmitter.py ${n_drones} cf" C-m
window=$((${window} + 1))

for ((i=0; i<${n_drones}; i++)); do
    tmux new-window -t $SESSION:${wiwndow} -n "Executor ${i}"
    tmux send-keys "python3 code/mission_receiver.py ${i} cf" C-m
    window=$((${window} + 1))
done

