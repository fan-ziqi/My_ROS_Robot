#!/bin/bash
echo "Starting Save Map File: $1"
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/ros_ws/src/nav/maps/pbstream/$1.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/ros_ws/src/nav/maps/$1 -pbstream_filename=${HOME}/ros_ws/src/nav/maps/pbstream/$1.pbstream -resolution=0.05

