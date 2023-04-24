#!/bin/bash

# Arguments
drone_namespace=$1
if [ -z "$drone_namespace" ]; then
    drone_namespace="cf"
fi
run_platform=$2
if [ -z "$run_platform" ]; then
    run_platform="false"
fi
using_optitrack=$3
if [ -z "$using_optitrack" ]; then
    using_optitrack="false"
fi

behavior_type="position" # "position" or "trajectory"
launch_bt="false" # "true" or "false"

source ./utils/launch_tools.bash

new_session $drone_namespace

new_window 'alphanumeric_viewer' "ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node \
    --ros-args -r  __ns:=/$drone_namespace"

if [[ "$run_platform" == "true" ]]
then
    new_window 'platform' "ros2 launch as2_platform_crazyflie crazyflie_swarm_launch.py \
        drone_id:=$drone_namespace \
        external_odom_topic:=self_localization/pose \
        external_odom:=$using_optitrack \
        estimator_type:=2 \
        controller_type:=1 \
        swarm_config_file:=config/crazy_swarm.yaml"

    if [[ "$using_optitrack" == "true" ]]
    then
        new_window 'mocap' "ros2 launch mocap_optitrack mocap.launch.py  \
        namespace:=optitrack \
        config_file:=config/mocap.yaml"
    fi
fi

new_window 'controller' "ros2 launch as2_motion_controller controller_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=false \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=false \
    plugin_name:=pid_speed_controller \
    plugin_config_file:=drone_config/controller.yaml"

if [[ "$using_optitrack" == "true" ]]
then
    new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
        namespace:=$drone_namespace \
        plugin_name:=mocap_pose"
else
    new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
        namespace:=$drone_namespace \
        plugin_name:=external_odom"
fi

new_window 'behaviors' "ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
    namespace:=$drone_namespace \
    follow_path_plugin_name:=follow_path_plugin_trajectory \
    go_to_plugin_name:=go_to_plugin_$behavior_type \
    takeoff_plugin_name:=takeoff_plugin_$behavior_type \
    land_plugin_name:=land_plugin_speed \
    follow_path_threshold:=0.3 \
    land_speed_condition_height:=0.2 \
    land_speed_condition_percentage:=0.4"

new_window 'traj_generator' "ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py  \
        namespace:=$drone_namespace"

# if [[ "$behavior_type" == "trajectory" ]]
# then
#     new_window 'traj_generator' "ros2 launch as2_behaviors_trajectory_generator dynamic_polynomial_generator_launch.py  \
#         namespace:=$drone_namespace"
# fi

if [[ "$launch_bt" == "true" ]] 
then
    new_window 'mission_planner' "ros2 launch as2_behavior_tree behaviour_trees.launch.py \
    drone_id:=$drone_namespace \
    groot_logger:=true \
    tree:=trees/go.xml"

    new_window 'groot' "$AEROSTACK2_WORKSPACE/build/groot/Groot --mode monitor"
fi

# echo -e "Launched drone $drone_namespace. For attaching to the session, run: \n  \t $ tmux a -t $drone_namespace"
