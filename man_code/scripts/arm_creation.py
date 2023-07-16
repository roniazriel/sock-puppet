#!/usr/bin/env python
from logging import warning
from datetime import datetime



def create_arm(interface_joints, joint_parent_axis, links, folder="folder"):
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
        #set the file name by it's configuration
        file_name += "_" + interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + links[i].replace(".", "_")
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
    # print(file_name)
    return file_name

class UrdfClass(object):
    """ this class create URDF files """

    def __init__(self, links=None, joints=None, joints_axis=None, rpy=None):
        """
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float]
        the first link will be the base link, who is always the same(connected to the world and link1  - his joint is limited to 0)
        """
        if rpy is None:
            rpy = []
        if joints_axis is None:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if joints is None:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if links is None:
            links = [1, 1, 1, 1, 1, 1]
        self.links = links
        self.joint_data = joints
        self.axis = self.init_calc(joints, joints_axis)
        self.links_number = len(self.links)
        self.rpy = rpy
        self.weights = self.calc_weight()

    def calc_weight(self):
        """
            defining mass value for each link
        """
        cumulative_length = 0
        cumulative_weight = 0
        link_mass = []
        for l in self.links:
            cumulative_length += float(l)
            mass = cumulative_length * 7.3149 + 1.1755 - cumulative_weight
            link_mass.append(str(mass))
            cumulative_weight += mass
        if len(link_mass) < 6: 
            while len(link_mass) < 6:
                fictive_mass = 0
                link_mass.append(str(fictive_mass))
        return link_mass

    # def calc_weight(self):
    #     """
    #     this function calculate the weight of the links according to accumilated weight and length of arm
    #     :return: weigths- the weight [kg] of each link - list of strings  (from the 2nd link)
    #     """
    #     coeffs = [7.3149, 1.1755]  # the coeffs of the linear eauation (found according UR5 and motoman)
    #     weights = [0]  # the wieght of each link
    #     acc_length = 0  # accumelated length
    #     acc_weight = 0  # accumelated weight
    #     for link in self.links[1:]:
    #         acc_length = acc_length+float(link)
    #         link_mass.append(round(acc_length*coeffs[0]+coeffs[1]-acc_weight,2))
    #         acc_weight = acc_weight + weights[-1]
    #     while len(weights) < 7:
    #         weights.append(1)
    #     return [str(weight) for weight in weights]

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
        <robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
        <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />
        <xacro:include filename="$(find man_gazebo)/urdf/''' + str(self.links_number) + '''dof/transmission_''' + str(
            self.links_number) + '''dof.xacro" />
        <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />

        <link name="world" />
        
        <joint name="world_joint" type="fixed">
            <parent link="world" />
            <child link = "base_link" />
            <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
        </joint>

        <xacro:macro name="cylinder_inertial" params="radius length mass *origin">
        <inertial>
          <mass value="${mass}" />
          <xacro:insert_block name="origin" />
          <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
            iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
            izz="${0.5 * mass * radius * radius}" />
        </inertial>
        </xacro:macro>

        <xacro:macro name="joint_limit" params="joint_type link_length ">
            <xacro:if value="${joint_type == 'revolute'}"  >
                <xacro:property name="joint_upper_limit" value="${pi}" />
                <xacro:property name="joint_lower_limit" value="${-pi}" />
            </xacro:if>
            <xacro:unless value="${joint_type == 'revolute'}"  >
                <xacro:property name="joint_upper_limit" value="${link_length}" />
                <xacro:property name="joint_lower_limit" value="${0}" />
            </xacro:unless>
        <limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
        </xacro:macro>

        <xacro:macro name="arm_robot" params="prefix ">'''

        inertia_parameters = '''
        <xacro:property name="base_radius" value="0.060" />
        <xacro:property name="base_length"  value="11" /> 
        <xacro:property name="base_height"  value="1" />
        <xacro:property name="base_width"  value="0.5" />
        <xacro:property name="base_mass" value="44" />


            <!-- Inertia parameters -->
        <xacro:property name="link0_mass" value="7" />
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="''' + self.weights[1] + '''" />
        <xacro:property name="link3_mass" value="''' + self.weights[2] + '''" />
        <xacro:property name="link4_mass" value="''' + self.weights[3] + '''" />
        <xacro:property name="link5_mass" value="''' + self.weights[4] + '''" />
        <xacro:property name="link6_mass" value="''' + self.weights[5] + '''" />

        <xacro:property name="link0_radius" value="0.060" /> 
        <xacro:property name="link1_radius" value="0.049" />
        <xacro:property name="link2_radius" value="0.045" />
        <xacro:property name="link3_radius" value="0.040" />
        <xacro:property name="link4_radius" value="0.035" />
        <xacro:property name="link5_radius" value="0.030" />
        <xacro:property name="link6_radius" value="0.025" /> '''

        base_link = '''

        <!-- Base Link -->
        <link name="${prefix}base_link" >
          <visual>
                <origin xyz="0 0 ${base_height/2}" rpy="0 0 0" /> 
            <geometry>
                    <box size="${base_width} ${base_length} ${base_height}"/> 
            </geometry>
          </visual>
          <collision>
                 <origin xyz="0 0 ${base_height/2}" rpy="0 0 0" /> 
            <geometry>
                    <box size="${base_width} ${base_length} ${base_height}"/>  
            </geometry>
          </collision>
          <inertial>
            <mass value="${base_mass}" />
            <origin xyz="0.0 0.0 ${base_height/2}" />
            <inertia ixx="436.5" ixy="0"  ixz="0"
                              iyy="1.8" iyz="0"
                                      izz="436.5"/>
          </inertial>
         
        </link>

        <xacro:property name="joint0_type" value="prismatic" /> 
        <xacro:property name="joint0_axe" value="0 1 0" /> 
        <xacro:property name="link0_length" value="0.1" />

        <!--  joint 0   -->
        <joint name="${prefix}joint0" type="${joint0_type}">
          <parent link="${prefix}base_link" />
          <child link = "${prefix}link0" />
          <origin xyz="0.0 0 ${base_height}" rpy="0 0.0 0" />
          <axis xyz="${joint0_axe}" />
          <xacro:joint_limit joint_type="${joint0_type}" link_length="${base_length/2}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>

         <!--  link 0 - The base of the robot  -->
        <link name="${prefix}link0">
          <visual>
            <origin xyz="0 0 ${link0_radius} " rpy="0 0 0" /> 
            <geometry>
                <cylinder radius="${link0_radius}" length="${link0_length}"/>    
            </geometry>
          </visual>
          <collision>
             <origin xyz="0 0 ${link0_radius}" rpy="0 0 0" /> 
            <geometry>
                <cylinder radius="${link0_radius}" length="${link0_length}"  mass="${link0_mass}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${link0_radius}" length="${link0_length}" mass="${link0_mass}">
            <origin xyz="0.0 0.0 ${link0_radius}" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link> 


    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''
        <!-- Sprayer joint - fictive joint -->
        <joint name="fake_joint" type="revolute">
            <parent link="${prefix}link''' + str(self.links_number) + '''" />
            <child link = "camera_link" />
            <origin xyz="0.0  0.0 ${link''' + str(self.links_number) + '''_length}" rpy="0.0 0.0 0" />
            <axis xyz="0 0 1"/>
            <xacro:joint_limit joint_type="revolute" link_length="0.1"/>
            <dynamics damping="0.0" friction="0.0"/>
        </joint>
        

        <!-- Sprayer -->
        <link name="camera_link">
          <collision>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
            <geometry>
            <box size="0.01 0.01 0.01"/>
            </geometry>
          </collision>
        <visual>
            <geometry>
            <box size="0.01 0.01 0.01"/>
            </geometry>
        </visual>
          <xacro:cylinder_inertial radius="0.01" length="0.01" mass="0.01">
            <origin xyz="0.0 0.0 0.005" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>

        <!-- ee joint -->
        <joint name="${prefix}ee_fixed_joint" type="fixed">
          <parent link="camera_link" />
          <child link = "${prefix}ee_link" />
          <origin xyz="0.0  0.0 0.01" rpy="0.0 0.0 0" />
        </joint>

            
        <!-- ee link -->
        <link name="${prefix}ee_link">
          <collision>
            <geometry>
              <box size="0.01 0.01 0.01"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
          </collision>
        </link>

        
            <xacro:arm_transmission prefix="${prefix}" />
            <xacro:arm_gazebo prefix="${prefix}" />
            </xacro:macro>
            <xacro:arm_robot prefix=""/>
        </robot>  '''

        txt = head + inertia_parameters + base_link + data + tail
        return txt

    @staticmethod
    def link_create(n):
        """link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity"""
        linkname = 'link' + str(n)
        link = ''
        if n == 1:
            link = link + '''<!--  link 1  -->
        <link name="${prefix}link1">
          <visual>
            <origin xyz="0 0 ${link1_length / 2} " rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link1_radius}" length="${link1_length}"/>
            </geometry>
          </visual>
          <collision>
             <origin xyz="0 0 ${link1_length / 2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link1_radius}" length="${link1_length}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
            <origin xyz="0.0 0.0 ${link1_length / 2}" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>'''
        else:
            link = link + '''<!-- link ''' + str(n) + '''   -->
        <link name="${prefix}''' + linkname + '''">
          <visual>
            <origin xyz="0 0 ${''' + linkname + '''_length / 2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
            </geometry>
          </visual>
          <collision>
            <origin xyz="0 0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
            </geometry>
          </collision>
          <xacro:cylinder_inertial radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
            <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>'''
        return link

    def calc_origin(self, n):
        # calc the origin of the link according to the previuos joint
        if self.joint_data[n - 1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                    return "0 0 ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['${1/2*pi} ', '0 ', '0 ']:  # links in the same directoin
                    return "0 -${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '${pi/2} ', '0 ']:
                    return "0 0 ${link" + str(n) + "_radius + link" + str(n - 1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
            else:  # pitch
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # around y: links are in the same directoin
                    return "0 ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} ${link" + str(
                        n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '0 ', '${-pi/2} ']:  # around x: links are not in the same directoin
                    return " ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} 0 ${link" + str(
                        n - 1) + "_length}"
                else:  # round x:  the links are perpendiculars
                    return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"

    def joint_create(self, n):
        jointname = 'joint' + str(n)

        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1    -->
        <joint name="${prefix}joint1" type="${joint1_type}">
          <parent link="${prefix}link0" />
          <child link="${prefix}link1" />
          <origin xyz="0.0 0.0 ${link0_length+0.011}" rpy="0.0 0.0 0.0" />
          <axis xyz="${joint1_axe}"/>
          <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>
    '''
        else:
            orgin = self.calc_origin(n)
            rpy = self.rpy[n - 1][0] + self.rpy[n - 1][1] + self.rpy[n - 1][2]
            joint = joint + '''<!--  joint ''' + str(n) + '''   -->
        <joint name="${prefix}''' + jointname + '''" type="${''' + jointname + '''_type}">
          <parent link="${prefix}link''' + str(n - 1) + '''"/>
          <child link="${prefix}link''' + str(n) + '''" />
          <origin xyz="''' + orgin + '''" rpy="''' + rpy + '''"/>
          <axis xyz="${''' + jointname + '''_axe}"/>
          <xacro:joint_limit joint_type="${''' + jointname + '''_type}" link_length="${link''' + str(n) + '''_length}"/>
          <dynamics damping="0.0" friction="0.0"/>
        </joint>
    '''
        return joint

    @staticmethod
    def urdf_write(data, filename = str(datetime.now())):
        path='/home/ar1/catkin_ws/src/sock-puppet/man_gazebo/urdf/6dof/arms/'
        fil = open(path+filename + '.urdf.xacro', 'w')
        fil.write(data)
        fil.close()

    def init_calc(self, joints, joints_axis):
        axis = []
        a = 0
        for j in joints:  # make calculations for all the joints
            axis.append(self.axis_calc(joints_axis[a]))
            a = a + 1
        return axis

    @staticmethod
    def axis_calc(axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            warning('wrong axe input.' + axe + ' entered. returning [0 0 0] ' + str(
                datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'




joint_types=['roll', 'pris', 'pitch', 'roll', 'pris', 'pitch']
joint_axis= ['z', 'y', 'x', 'x', 'y', 'y']
links =['0.1', '0.3', '0.5', '0.1', '0.7', '0.3']
create_arm(joint_types,joint_axis,links)

#pso
# ['roll', 'pris', 'pitch', 'pitch', 'roll', 'pris']  ['z', 'y', 'z', 'y', 'x', 'y']  ['0.1', '0.5', '0.3', '0.1', '0.3', '0.1']
# ['roll', 'pitch', 'pris', 'pitch', 'roll', 'roll']  ['z', 'y', 'z', 'y', 'z', 'y']  ['0.1', '0.1', '0.7', '0.3', '0.5', '0.3']
# ['roll', 'roll', 'roll', 'pris', 'pitch', 'roll']   ['z', 'y', 'y', 'z', 'y', 'x']  ['0.1', '0.3', '0.5', '0.7', '0.3', '0.1']
# ['roll', 'roll', 'roll', 'pris', 'roll', 'roll']    ['z', 'y', 'y', 'y', 'y', 'y']  ['0.1', '0.3', '0.5', '0.7', '0.1', '0.1']
# ['roll', 'pris', 'pitch', 'pitch', 'roll', 'pris']  ['z', 'y', 'x', 'y', 'x', 'y']  ['0.1', '0.7', '0.3', '0.3', '0.1', '0.5']
# ['roll', 'pitch', 'pitch', 'pris', 'pitch', 'roll'] ['z', 'y', 'x', 'z', 'y', 'x']  ['0.1', '0.3', '0.5', '0.7', '0.3', '0.1']
# ['roll', 'pris', 'pitch', 'roll', 'pitch', 'roll']  ['z', 'y', 'x', 'z', 'y', 'x']  ['0.1', '0.5', '0.5', '0.7', '0.1', '0.1']
# ['roll', 'pris', 'pris', 'roll', 'pitch', 'pitch']  ['z', 'y', 'y', 'x', 'y', 'y']  ['0.1', '0.3', '0.1', '0.7', '0.7', '0.1']
# ['roll', 'pitch', 'roll', 'pris', 'pitch', 'pitch'] ['z', 'y', 'x', 'z', 'y', 'z']  ['0.1', '0.1', '0.5', '0.7', '0.1', '0.1']
# ['roll', 'pitch', 'pitch', 'pris', 'pitch', 'roll'] ['z', 'y', 'x', 'z', 'y', 'x']  ['0.1', '0.3', '0.7', '0.3', '0.1', '0.1']

#pso+xgboost
# ['roll', 'pris', 'pitch', 'pris', 'pitch', 'pitch'] ['z', 'y', 'z', 'x', 'y', 'z']  ['0.1', '0.5', '0.1', '0.7', '0.3', '0.1']
# ['roll', 'pitch', 'pris', 'pris', 'roll', 'pitch']  ['z', 'y', 'y', 'x', 'z', 'y']  ['0.1', '0.1', '0.5', '0.5', '0.7', '0.1']
# ['roll', 'pris', 'pitch', 'pitch', 'pitch', 'pitch']    ['z', 'y', 'x', 'z', 'z', 'x']  ['0.1', '0.3', '0.7', '0.1', '0.7', '0.1']
# ['roll', 'roll', 'roll', 'pris', 'pitch', 'roll']   ['z', 'y', 'y', 'z', 'y', 'x']  ['0.1', '0.3', '0.5', '0.7', '0.1', '0.1']
# ['roll', 'roll', 'pitch', 'pris', 'pitch', 'roll']  ['z', 'y', 'y', 'z', 'y', 'z']  ['0.1', '0.7', '0.7', '0.1', '0.1', '0.3']
# ['roll', 'pitch', 'pitch', 'pris', 'roll', 'roll']  ['z', 'y', 'x', 'z', 'y', 'y']  ['0.1', '0.3', '0.7', '0.5', '0.1', '0.1']
# ['roll', 'roll', 'pitch', 'pris', 'roll', 'roll']   ['z', 'y', 'y', 'z', 'y', 'y']  ['0.1', '0.7', '0.1', '0.7', '0.1', '0.1']
# ['roll', 'pitch', 'roll', 'pris', 'pitch', 'pitch'] ['z', 'y', 'z', 'z', 'y', 'x']  ['0.1', '0.1', '0.5', '0.7', '0.1', '0.1']
# ['roll', 'pitch', 'pitch', 'pris', 'pitch', 'pris'] ['z', 'y', 'z', 'x', 'y', 'x']  ['0.1', '0.3', '0.1', '0.7', '0.3', '0.5']
# ['roll', 'pris', 'pitch', 'roll', 'pris', 'pitch']  ['z', 'y', 'x', 'x', 'y', 'y']  ['0.1', '0.3', '0.5', '0.1', '0.7', '0.3']
