#!/bin/bash

config_path="$1"
config_path=${config_path:="simulation_config/swarm4.json"}

./launch_ignition.bash ${config_path}
