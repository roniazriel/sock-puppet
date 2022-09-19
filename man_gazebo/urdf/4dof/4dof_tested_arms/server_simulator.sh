#!/bin/bash

dir_size=2000
dir_name="/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/4dof_tested_arms"
alias proj="/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/4dof_tested_arms"
#cd /home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/4dof/4dof_tested_arms
n=$((`find . -maxdepth 1 -type f | wc -l`/$dir_size+1))
for i in `seq 1 $n`;
do
    mkdir -p "$dir_name$i";
    find . -maxdepth 1 -type f | head -n $dir_size | xargs -i mv "{}" "$dir_name$i"
done

