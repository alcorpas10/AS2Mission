#!/bin/bash

ros2 topic pub -1 /planner/notification/drone_events mutac_msgs/msg/State "{identifier: {natural: $1}, state: 1, position: {x: $(expr $1 - 1), y: -1, z: 1}, type: 11}"
