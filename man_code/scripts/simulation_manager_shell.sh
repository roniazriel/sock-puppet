#!/bin/bash

jointIndex=$1 # folder of robotic
linkIndex=$2
ArmName=$3
resfile=$4

source /home/ar1/catkin_ws/devel/setup.bash # added for server experiment

python ~/catkin_ws/src/sock-puppet/man_code/scripts/simulator.py $jointIndex $linkIndex $ArmName $resfile

echo "Done"



