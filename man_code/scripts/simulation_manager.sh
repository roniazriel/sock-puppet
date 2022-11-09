#!/bin/bash


FILES=$1
directory=$2
TOdirectory=$3
RESULT=$4
DOF=$5
COUNTER=0
echo "$FILES"
echo "$directory"
echo "$TOdirectory"
echo "$RESULT"
echo "$DOF"


#FILES=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/*
#directory=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/
# TOdirectory=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/4dof_tested_arms/
# RESULT=~/catkin_ws/src/sock-puppet/results/checkserver.csv
# COUNTER=0
# DOF=4

for FILE in "$FILES"/*  
do 
   start=`date +%s`
   filename=$(basename -- "$FILE")
   extension="${filename##*.}"
   filename="${filename%.*}"
   echo "$filename"
   python ~/catkin_ws/src/sock-puppet/man_code/scripts/simulator.py $filename $directory $TOdirectory $RESULT $DOF $COUNTER
   end=`date +%s`
   runtime=$((end-start))
   echo "Done"
   echo "$runtime"
   let COUNTER++
done

