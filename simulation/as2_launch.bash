#!/bin/bash

if [ "$#" -le 0 ]; then
	echo "usage: $0 [drone_namespace] "
	exit 1
fi

simulation_config=$2
if [ -z "$simulation_config" ]; then
    simulation_config=""
fi

launch_keyboard_teleop=$3
if [ -z "$launch_keyboard_teleop" ]; then
    launch_keyboard_teleop="false"
fi

# Arguments
drone_namespace=$1
use_sim_time=true
behavior_type="position" # "position" or "trajectory"

source ./utils/launch_tools.bash

new_session $drone_namespace

new_window 'alphanumeric_viewer' "ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node \
    --ros-args -r  __ns:=/$drone_namespace"

new_window 'platform' "ros2 launch as2_platform_ign_gazebo ign_gazebo_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    config_file:=$simulation_config"

new_window 'controller' "ros2 launch as2_motion_controller controller_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=true \
    plugin_name:=pid_speed_controller \
    plugin_config_file:=config/controller.yaml"

new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    plugin_name:=ground_truth \
    plugin_config_file:=config/default_state_estimator.yaml" 

new_window 'behaviors' "ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    follow_path_plugin_name:=follow_path_plugin_$behavior_type \
    go_to_plugin_name:=go_to_plugin_$behavior_type \
    go_to_threshold:=0.2 \
    takeoff_plugin_name:=takeoff_plugin_$behavior_type \
    land_plugin_name:=land_plugin_speed"

if [[ "$behavior_type" == "trajectory" ]]
then
    new_window 'traj_generator' "ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py  \
        namespace:=$drone_namespace \
        use_sim_time:=$use_sim_time"
fi

if [[ "$launch_keyboard_teleop" == "true" ]]
then
    new_window 'keyboard_teleop' "ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py  \
        namespace:=$drone_namespace \
        use_sim_time:=$use_sim_time"
fi
