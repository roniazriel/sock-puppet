#!/bin/bash


FILES=~/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/arms/*
COUNTER=0
for FILE in $FILES  
do 
   start=`date +%s`
   filename=$(basename -- "$FILE")
   extension="${filename##*.}"
   filename="${filename%.*}"
   echo "$filename"
   python ~/catkin_ws/src/sock-puppet/man_code/scripts/simulator.py $filename $COUNTER
   end=`date +%s`
   runtime=$((end-start))
   echo "Done"
   echo "$runtime"
   let COUNTER++
done

