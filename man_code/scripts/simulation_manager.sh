#!/bin/bash


FILES=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/*
directory=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/
TOdirectory=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/4dof_tested_arms/
RESULT=~/catkin_ws/src/sock-puppet/results/checkserver.csv
COUNTER=0
DOF=4
for FILE in $FILES  
do 
   start=`date +%s`
   filename=$(basename -- "$FILE")
   extension="${filename##*.}"
   filename="${filename%.*}"
   echo "$filename"
   /usr/bin/python2.7 ~/catkin_ws/src/sock-puppet/man_code/scripts/simulator.py $filename $COUNTER $directory $TOdirectory $RESULT $DOF
   end=`date +%s`
   runtime=$((end-start))
   echo "Done"
   echo "$runtime"
   let COUNTER++
done

