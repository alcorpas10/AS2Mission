#!/bin/bash

ros2 topic pub -1 /planner/signal/drone_alarm mutac_msgs/msg/Alarm "{identifier: {natural: $1}, alarm: 1}"
