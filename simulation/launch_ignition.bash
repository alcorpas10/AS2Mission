#!/bin/bash

config_path="$1"
config_path=${config_path:="simulation_config/default.json"}

export RUN_ON_START=1

SCRIPT_PATH="${AEROSTACK2_PATH}/as2_simulation_assets/as2_ign_gazebo_assets/scripts"
ros2 launch as2_ign_gazebo_assets launch_simulation.py config_file:=${config_path} use_sim_time:=true
