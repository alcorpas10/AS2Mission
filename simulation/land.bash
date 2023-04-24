#!/bin/bash

ros2 topic pub -1 /emergency_land std_msgs/msg/Int32 "data: $1"
