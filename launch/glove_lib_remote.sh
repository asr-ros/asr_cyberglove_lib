#!/bin/bash
export ROS_MASTER_URI="http://$HOSTNAME:11311"
export ROSLAUNCH_SSH_UNKNOWN=1
roslaunch asr_cyberglove_lib glove_lib_remote.launch
