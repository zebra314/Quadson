#!/bin/zsh

# Compile the project
source /opt/ros/noetic/setup.zsh
catkin_make
source /root/quadson_ws/devel/setup.zsh
cd /root/quadson_ws

exec "$@"
