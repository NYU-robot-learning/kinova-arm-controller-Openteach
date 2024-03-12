#! /usr/bin/python3
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys

import math

import actionlib
import kinova_msgs.msg


""" Global variable """
robot_category = 'j'
robot_category_version = 2
wrist_type = 'n'
arm_joint_number = 6
robot_mode = 's'
prefix = 'j2n6s300_'
currentJointCommand = [] # number of joints is defined in __main__

def joint_angle_client(angle_set):
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
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('the joint angle action timed-out')
        client.cancel_all_goals()
        return None


def getcurrentJointCommand():
    # wait to get current position
    topic_address = '/j2n6s300_driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)

def setcurrentJointCommand(feedback):
    global currentJointCommand

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])


def unitParser_joint(joint_value, relative_):
    """ Argument unit """
    global currentJointCommand

    joint_degree_command = list(map(math.degrees, joint_value))
    # get absolute value
    if relative_:
        joint_degree_absolute_ = [joint_degree_command[i] + currentJointCommand[i] for i in range(0, len(joint_value))]
    else:
        joint_degree_absolute_ = joint_degree_command
    joint_degree = joint_degree_absolute_

    return joint_degree

if __name__ == '__main__':

    rospy.init_node('j2n6s300_gripper_workout')

    # KinovaType defines AngularInfo has 7DOF, so for published topics on joints.
    currentJointCommand = [0] * 7

    # get Current finger position if relative position
    getcurrentJointCommand()
    joint_degree = unitParser_joint('radian', [1, 0, 0, 0, 0, 0], True)

    positions = [0]*7
    try:

        if arm_joint_number < 1:
            print('Joint number is 0, check with "-h" to see how to use this node.')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            sys.exit() 
        else:
            for i in range(0,arm_joint_number):
              positions[i] = joint_degree[i]               

        result = joint_angle_client(positions)

    except rospy.ROSInterruptException:
        print('program interrupted before completion')