from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
from simulation import *
import roslaunch
from Spray import Spray
import os



def ter_command(command):
	#"""Write Command to the terminal"""
    try:
        command = shlex.split(command)
        ter_command_proc = subprocess.Popen(command, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        return ter_command_proc
    except ValueError:
        rospy.loginfo('Error occurred at ter_command function')  # shows warning message
        pass


def checkroscorerun():
    try:
        roscore_pid = rosgraph.Master('/rostopic').getPid()
        return roscore_pid
    except error as e:
        pass

def start_launch(launch_name, pathname, launch_path, args=None):
    """Start launch file"""
    if args is None:
        args = []
    try:
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        path = pathname+launch_path+"/launch/"+launch_name+".launch"
        cli_args = [path, args]

        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        # sleep(50)
        return launch
    except ValueError:
        rospy.loginfo('Error occurred at start launch function')
        pass


def stop_launch(launch):
    try:
        launch.shutdown()
        return False
    except ValueError:
        rospy.loginfo('Error occurred at launch_stop function')
        pass


def create_arm(interface_joints, joint_parent_axis, links, folder):
    """create the desired arm
        interface_joints- roll,pitch,yaw or prismatic
                         roll - revolute around own Z axe
                         pitch - revolute that not roll
                         pris - prismatic along
        links - length of links
        joint_parent_axis - the axe, in the parent frame, which each joint use
    """
    joints = []
    joint_axis = []
    rpy = []
    file_name = ""
    rolly_number = -1
    pitchz_number = 1
    prisy_number = -1
    for i in range(len(joint_parent_axis)):
        file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + links[i].replace(".", "_")
        if interface_joints[i].replace(" ", "") == "roll":
            joints.append("revolute")
            joint_axis.append('z')
            if joint_parent_axis[i].replace(" ", "") == "y":
                # rpy.append(['${-pi/2} ', '0 ', '0 '])
                rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                rpy.append([rolly_rot, '0 ', '0 '])
                rolly_number = rolly_number * -1
            elif joint_parent_axis[i].replace(" ", "") == "x":
                rpy.append(['0 ', '${pi/2} ', '0 '])
            elif joint_parent_axis[i].replace(" ", "") == "z":
                rpy.append(['0 ', '0 ', '0 '])
        elif interface_joints[i].replace(" ", "") == "pitch":
            joints.append("revolute")
            joint_axis.append('y')
            if joint_parent_axis[i].strip() == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i].strip() == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i].strip() == "z":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                pitchz = '${' + str(pitchz_number) + '/2*pi} '
                rpy.append([pitchz, '0 ', '0 '])
                pitchz_number = pitchz_number * -1
        elif interface_joints[i].replace(" ", "") == "pris":
            joints.append("prismatic")
            joint_axis.append('z')
            if joint_parent_axis[i].strip() == "y":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                prisy = '${' + str(prisy_number) + '/2*pi} '
                rpy.append([prisy, '0 ', '0 '])
                prisy_number = prisy_number * -1
            elif joint_parent_axis[i].strip() == "x":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
            elif joint_parent_axis[i].strip() == "z":
                rpy.append(['0 ', '0 ', '0 '])
    arm = UrdfClass(links, joints, joint_axis, rpy)
    arm.urdf_write(arm.urdf_data(), file_name)
    return {"arm": arm, "name": file_name, "folder": folder}


path = os.environ['HOME'] + "/catkin_ws/src/sock-puppet/"  
# Generate URDF
interface_joints =["roll","roll", "roll", "roll", "roll", "roll"]
joint_parent_axis =['z', 'y', 'y', 'y', 'y', 'z']
links = ['0.1', '0.7', '0.4', '0.7', '0.1', '0.6']
file_name = "test_arm"
folder = "folder"

arm = create_arm(interface_joints, joint_parent_axis, links, folder)
arm_name = arm["name"]
print(arm_name)


# Launch URDF Model
main_launch_arg = ["gazebo_gui:=true", "rviz:=true", "dof:=" + "6" + "dof", "man:= manipulator"]
launch= start_launch("main", path, "man_gazebo", main_launch_arg)  # main launch file


# spray 10 clusters
# and take measurments
spray= Spray()
spray.start_spray()

# kill proccess
launch.shutdown()

