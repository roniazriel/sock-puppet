#!/bin/bash
cd ~/catkin_ws/src/sock-puppet/man_gazebo/urdf/5dof/arms
if [ -e _roll_z_0_1_pris_y_0_3_roll_x_0_3_pris_z_0_7_pris_y_0_5.urdf.xacro ]
then
    echo "ok"
else
    echo "nok"
fi
