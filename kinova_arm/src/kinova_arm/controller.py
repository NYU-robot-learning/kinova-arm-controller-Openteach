#! /usr/bin/python3
import roslib; roslib.load_manifest('kinova_arm')
import rospy
import actionlib
import kinova_msgs.msg
import std_msgs.msg 
from sensor_msgs.msg import JointState
import geometry_msgs.msg
import math
import numpy as np

class KinovaController(object):

    def __init__(self):
        
        try:
            rospy.init_node('j2n6s300_node')
        except:
            pass

        self.current_joint_pose = None
        self.arm_joint_number = 6
        self.prefix = 'j2n6s300_'
        self.currentJointCommand = [0]*7
        self.currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq

        rospy.Subscriber('/j2n6s300_driver/out/joint_state', JointState, self._sub_callback_joint_state)

        self.velocity_publisher = rospy.Publisher(
            '/j2n6s300_driver/in/cartesian_velocity',
            kinova_msgs.msg.PoseVelocity,
            queue_size = 1
        )

    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def joint_angle_client(self, angle_set):
        """Send a joint angle goal to the action server."""
        action_address = '/j2n6s300_driver/joints_action/joint_angles'
        client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.ArmJointAnglesAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmJointAnglesGoal()

        goal.angles.joint1 = angle_set[0]
        goal.angles.joint2 = angle_set[1]
        goal.angles.joint3 = angle_set[2]
        goal.angles.joint4 = angle_set[3]
        goal.angles.joint5 = angle_set[4]
        goal.angles.joint6 = angle_set[5]
        goal.angles.joint7 = angle_set[6]

        client.send_goal(goal)

    def cartesian_pose_client(self, position, orientation):
        action_address = '/j2n6s300_driver/pose_action/tool_pose'
        client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=('j2n6s300_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        client.send_goal(goal)

    def EulerXYZ2Quaternion(self, EulerXYZ_):
        tx_, ty_, tz_ = EulerXYZ_[0:3]
        sx = math.sin(0.5 * tx_)
        cx = math.cos(0.5 * tx_)
        sy = math.sin(0.5 * ty_)
        cy = math.cos(0.5 * ty_)
        sz = math.sin(0.5 * tz_)
        cz = math.cos(0.5 * tz_)

        qx_ = sx * cy * cz + cx * sy * sz
        qy_ = -sx * cy * sz + cx * sy * cz
        qz_ = sx * sy * cz + cx * cy * sz
        qw_ = -sx * sy * sz + cx * cy * cz

        Q_ = [qx_, qy_, qz_, qw_]
        return Q_

    def getcurrentCartesianCommand(self):
        # wait to get current position
        topic_address = '/j2n6s300_driver/out/cartesian_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)

    def setcurrentCartesianCommand(self, feedback):
        global currentCartesianCommand

        currentCartesianCommand_str_list = str(feedback).split("\n")

        for index in range(0,len(currentCartesianCommand_str_list)):
            temp_str=currentCartesianCommand_str_list[index].split(": ")
            self.currentCartesianCommand[index] = float(temp_str[1])

    def getcurrentJointCommand(self):
        # wait to get current position
        topic_address = '/j2n6s300_driver/out/joint_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, self.setcurrentJointCommand)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)

    def setcurrentJointCommand(self, feedback):

        currentJointCommand_str_list = str(feedback).split("\n")
        for index in range(0,len(currentJointCommand_str_list)):
            temp_str=currentJointCommand_str_list[index].split(": ")
            self.currentJointCommand[index] = float(temp_str[1])

    def unitParser_cartesian(self, pose_value_, relative_, is_quaternion_):
        """ Argument unit """
        global currentCartesianCommand

        position_ = pose_value_[:3]
        orientation_ = pose_value_[3:]

        for i in range(0,3):
            if relative_:
                position_[i] = pose_value_[i] + self.currentCartesianCommand[i]
            else:
                position_[i] = pose_value_[i]

        if relative_:
                orientation_rad_list =  self.currentCartesianCommand[3:]
                orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
        else:
            orientation_rad = orientation_

        if is_quaternion_:
            orientation_q = orientation_rad
        else:    
            orientation_q = self.EulerXYZ2Quaternion(orientation_rad)

        # pose_mq_ = position_ + orientation_q
        pose_mq_ = np.concatenate([position_, orientation_q], axis=0)

        return pose_mq_

    def unitParser_joint(self, joint_value, relative_):
        """ Argument unit """
        global currentJointCommand

        joint_degree_command = list(map(math.degrees, joint_value))
        # get absolute value
        if relative_:
            joint_degree_absolute_ = [joint_degree_command[i] + self.currentJointCommand[i] for i in range(0, len(joint_value))]
        else:
            joint_degree_absolute_ = joint_degree_command
        joint_degree = joint_degree_absolute_

        return joint_degree


    def cartesian_movement(self, input_array, relative, is_quaternion):

        self.getcurrentCartesianCommand()

        pose_mq = self.unitParser_cartesian(input_array, relative, is_quaternion)

        try:
            poses = [float(n) for n in pose_mq]
            self.cartesian_pose_client(poses[:3], poses[3:])

        except rospy.ROSInterruptException:
            print("program interrupted before completion")

    def joint_movement(self, input_array, relative):

        # get Current finger position if relative position
        self.getcurrentJointCommand()
        joint_degree = self.unitParser_joint(input_array, relative)

        positions = [0]*7

        try:
            for i in range(0, self.arm_joint_number):
                positions[i] = joint_degree[i]               

            self.joint_angle_client(positions)

        except rospy.ROSInterruptException:
            print('program interrupted before completion')

    def move_home(self):

        self.getcurrentJointCommand()
        joint_degree = self.unitParser_joint([4.80750942, 2.92150663, 1, -2.085, 1.4470525, 7.60178156], False)

        positions = [0]*7

        try:
            for i in range(0, self.arm_joint_number):
                positions[i] = joint_degree[i]               

            self.joint_angle_client(positions)

        except rospy.ROSInterruptException:
            print('program interrupted before completion')

    def get_current_position(self):
        if self.current_joint_pose == None:
            print('No joint pose read!')
            return

        print(self.current_joint_pose.position)

    def publish_cartesian_velocity(self, velocity_vector, duration):
        velocity_command = kinova_msgs.msg.PoseVelocity()
        velocity_command.twist_linear_x = velocity_vector[0];
        velocity_command.twist_linear_y = velocity_vector[1];
        velocity_command.twist_linear_z = velocity_vector[2];
        velocity_command.twist_angular_x = velocity_vector[3];
        velocity_command.twist_angular_y = velocity_vector[4];
        velocity_command.twist_angular_z = velocity_vector[5];

        step = 0 
        rate = rospy.Rate(100) # Velocity control frequency
        while step < 100 * duration:
            self.velocity_publisher.publish(velocity_command)
            step = step + 1
            rate.sleep()