#!/bin/bash

SESSION="Mision"

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:1 -n 'Monitor 0'
tmux send-keys "ros2 launch monitor execution_monitor.py drone_id:=0 config_file:=config/config_prueba_swarm2.yaml" C-m 

tmux new-window -t $SESSION:2 -n 'Monitor 1'
tmux send-keys "ros2 launch monitor execution_monitor.py drone_id:=1 config_file:=config/config_prueba_swarm2.yaml" C-m

tmux new-window -t $SESSION:3 -n 'Monitor 2'
tmux send-keys "ros2 launch monitor execution_monitor.py drone_id:=2 config_file:=config/config_prueba_swarm2.yaml" C-m

tmux new-window -t $SESSION:4 -n 'Replanner'
tmux send-keys "ros2 launch replanning_manager replanning_manager.py" C-m

tmux new-window -t $SESSION:5 -n 'Viewer'
tmux send-keys "ros2 launch viewer viewer.py config_file:=config/config_prueba_swarm2.yaml" C-m

tmux new-window -t $SESSION:6 -n 'Mission Transmitter 0'
tmux send-keys "python3 mission_transmitter.py 3 cf" C-m

tmux new-window -t $SESSION:7 -n 'Mission Reciever 0'
tmux send-keys "python3 mission_reciever.py 0" C-m

tmux new-window -t $SESSION:8 -n 'Mission Reciever 1'
tmux send-keys "python3 mission_reciever.py 1" C-m

tmux new-window -t $SESSION:9 -n 'Mission Reciever 2'
tmux send-keys "python3 mission_reciever.py 2" C-m
