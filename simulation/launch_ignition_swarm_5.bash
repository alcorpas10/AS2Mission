#!/bin/bash

config_path="$1"
config_path=${config_path:="simulation_config/swarm5.json"}

./launch_ignition.bash ${config_path}
