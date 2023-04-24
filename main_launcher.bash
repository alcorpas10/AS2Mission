#!/bin/bash

# bash script to run the program
# Arguments
drone_namespaces=('cf0' 'cf1' 'cf2')
# drone_namespaces=('cf0')

using_optitrack="true"

# Run the program with index 0
./launch_as2.bash "${drone_namespaces[0]}" true $using_optitrack
# Run the program with index 1
./launch_as2.bash "${drone_namespaces[1]}" false $using_optitrack
# Run the program with index 2
./launch_as2.bash "${drone_namespaces[2]}" false $using_optitrack


# Run the program with index 1
# ./launch_as2.bash "${drone_namespaces[0]}" true $using_optitrack

session=${drone_namespace[0]}

# if inside a tmux session detach before attaching to the session
if [ -n "$TMUX" ]; then
    tmux switch-client -t $session
else
    tmux attach -t $session:0
fi
