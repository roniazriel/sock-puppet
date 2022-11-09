#!/bin/bash


RESULT=$1


source /home/ar1/catkin_ws/devel/setup.bash # added for server experiment

COUNTER=0
echo "$FILES"
echo "$directory"
echo "$TOdirectory"
echo "$RESULT"
echo "$DOF"

python ~/catkin_ws/src/sock-puppet/man_code/scripts/fake_sim.py $RESULT 