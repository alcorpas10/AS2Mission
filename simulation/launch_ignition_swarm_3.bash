#!/bin/bash

config_path="$1"
config_path=${config_path:="simulation_config/swarm3.json"}

./launch_ignition.bash ${config_path}
