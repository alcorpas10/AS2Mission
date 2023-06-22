#!/bin/bash

usage() {
    echo "  options:"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -m: multi agent, choices: [number >= 1]"
    echo "      -e: estimator_type, choices: [ground_truth, raw_odometry, mocap_pose]"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespace, default is cf"
    echo "      -o: open extra nodes, yml file"
    echo "      -x: dont execute as2 and gazebo"
}

# Arg parser
while getopts "sm:e:rtn:o:x" opt; do
  case ${opt} in
    s )
      simulated="true"
      ;;
    m )
      swarm="${OPTARG}"
      ;;
    e )
      estimator_plugin="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    n )
      drone_namespace="${OPTARG}"
      ;;
    o )
      open_yml="${OPTARG}"
      ;;
    x )
      no_execute="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

source utils/tools.bash

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
simulated=${simulated:="false"}  # default ign_gz
if [[ ${simulated} == "false" && -z ${estimator_plugin} ]]; then
  echo "Error: when -s is false, -e argument must be set" 1>&2
  usage
  exit 1
fi

no_execute=${no_execute:="false"}

swarm=${swarm:=1}
estimator_plugin=${estimator_plugin:="ground_truth"}  # default ign_gz
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
drone_namespace=${drone_namespace:="cf"}

# if [[ ${swarm} != 1 ]]; then
#   num_drones=${swarm}
#   simulation_config="sim_config/world_swarm.json"
# else
#   num_drones=1
#   simulation_config="sim_config/world.json"
# fi
num_drones=$((${swarm}))
simulation_config="sim_config/swarm${num_drones}.json"

echo "open_yml = ${open_yml}"

open_yml=${open_yml:=""}

echo "Num drones: ${num_drones}"

# Generate the list of drone namespaces
drone_ns=()
for ((i=0; i<${num_drones}; i++)); do
  drone_ns+=("$drone_namespace$i")
done

if [[ ${no_execute} == "false" ]]; then
  for ns in "${drone_ns[@]}"; do
    tmuxinator start -n ${ns} -p utils/session.yml drone_namespace=${ns} base_launch=false estimator_plugin=${estimator_plugin} simulation=${simulated} simulation_config=${simulation_config} &
    wait
  done

  if [[ ${estimator_plugin} == "mocap_pose" ]]; then
    tmuxinator start -n mocap -p utils/mocap.yml &
    wait
  fi

  if [[ ${simulated} == "true" ]]; then
    tmuxinator start -n gazebo -p utils/gazebo.yml simulation_config=${simulation_config} &
    wait
  fi
fi

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p utils/rosbag.yml drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p utils/keyboard_teleop.yml simulation=true drone_namespace=$(list_to_string "${drone_ns[@]}") &
  wait
fi

tmuxinator start -n mission -p utils/empty.yml &
wait

echo "open_yml = ${open_yml}"

if [[ ${open_yml} != "" ]]; then
  open_yml=utils/${open_yml}.yml
  echo "Opening ${open_yml}"
  tmuxinator start -n open -p ${open_yml} n_drones=${num_drones} &
  wait
fi

tmux attach-session -t mission:mission
