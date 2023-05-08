#!/bin/bash

ros2 topic pub -1 /planner/signal/emergency_land std_msgs/msg/Int32 "data: $1"
