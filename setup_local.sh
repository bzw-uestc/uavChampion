#!/bin/bash

source devel/setup.bash
roslaunch uav_control run_all.launch & sleep 2
roslaunch uav_control run_vins_local.launch &
sleep 0.1
wait
exit 