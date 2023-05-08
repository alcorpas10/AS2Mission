#!/bin/bash
ros2 bag record -o $1 /tf /tf_static /clock /planner/visualization/cf0/path_left /planner/visualization/cf0/covered_path /planner/visualization/cf1/path_left /planner/visualization/cf1/covered_path /planner/visualization/cf2/path_left /planner/visualization/cf2/covered_path
