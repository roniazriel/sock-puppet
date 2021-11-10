#!/usr/bin/env python
from Simulation import *



def to_urdf(interface_joints, joint_parent_axis, links, folder):
    """Create the desired confiuration
    :param interface_joints- [list] roll,pitch or prismatic (roll -revolute around own Z axis,
                                    pitch - revolute that not roll,  pris - prismatic along z)
    :param links -[list] length of links
    :param joint_parent_axis - [list] the axe, in the parent frame, which each joint use
    :param folder - [str] where the urdf saved - not in use
    :return -[dict] -contain the configuration name and all the data to the urdf file
        """
    joints = []
    joint_axis = []
    rpy = []
    # file_name = os.environ['HOME'] + "\Tamir_Ws\src\manipulator_ros\Manipulator\man_gazebo\urdf\"
    # + str(dof) + "dof\combined\"
    file_name = ""
    rolly_number = -1
    pitchz_number = 1
    prisy_number = -1
    for i in range(len(joint_parent_axis)):
        # file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + \
        #              links[i].replace(".", "_")
        file_name += interface_joints[i] + "_" + joint_parent_axis[i] + "_" + str(links[i]).replace(".", "_")
        if interface_joints[i] == "roll":
            joints.append("revolute")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                rpy.append([rolly_rot, '0 ', '0 '])
                rolly_number = rolly_number * -1
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                rpy.append(['0 ', '0 ', '0 '])
        elif interface_joints[i] == "pitch":
            joints.append("revolute")
            joint_axis.append('y')
            if joint_parent_axis[i] == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i] == "z":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                pitchz = '${' + str(pitchz_number) + '/2*pi} '
                rpy.append([pitchz, '0 ', '0 '])
                pitchz_number = pitchz_number * -1
        elif interface_joints[i] == "pris":
            joints.append("prismatic")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                prisy = '${' + str(prisy_number) + '/2*pi} '
                rpy.append([prisy, '0 ', '0 '])
                prisy_number = prisy_number * -1
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                rpy.append(['0 ', '0 ', '0 '])
    arm = UrdfClass(links, joints, joint_axis, rpy)
    arm.urdf_write(arm.urdf_data(), file_name)
    return {"arm": arm, "name": file_name, "folder": folder}
            
 

# Sampling algorithm
interface_joints =["pitch","pitch", "pitch", "pitch", "roll", "pitch"]
joint_parent_axis =['z', 'z', 'y', 'x', 'z', 'y']
links = ['0.44', '0.75', '0.3', '0.8', '0.1', '0.07']
file_name = "test_arm"
folder = "folder"

arm = to_urdf(interface_joints, joint_parent_axis, links, folder)
arm_name = arm["name"]
print(arm_name)


