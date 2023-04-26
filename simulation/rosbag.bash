#!/bin/bash
ros2 bag record -o $1 /tf /tf_static /clock /path0_left /path0_covered /path1_left /path1_covered /path2_left /path2_covered
