#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 15 22:45:20 2014

@author: Sam Pfeiffer

Put robot in pre-established known pose Home which is recommended
for navigation in narrow spaces (i.e.: crossings doors)

"""
import rospy
import actionlib
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, MoveItErrorCodes, JointConstraint
from moveit_joints_goal import create_move_group_joints_goal

# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

def send_REEM_to_home_pose():
    try:
        rospy.init_node("send_robot_to_home")
    except rospy.exceptions.ROSException: # if the node is already initialized this gives exception
        pass

    rospy.logdebug("Connecting to move_group AS")
    moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    moveit_ac.wait_for_server()
    rospy.logdebug("Succesfully connected.")
    
    rospy.loginfo("Sending REEM to home position.")
    
    joint_names = ['head_1_joint', 'head_2_joint', 'torso_1_joint', 'torso_2_joint',
                   'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                   'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                   'arm_right_7_joint', 'arm_left_1_joint', 'arm_left_2_joint',
                   'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint',
                   'arm_left_6_joint', 'arm_left_7_joint']
    # home pose
    joint_list_home = len(joint_names)*[0.0]

    moveit_goal = create_move_group_joints_goal(joint_names, joint_list_home, 
                                                group="both_arms_and_head", plan_only=False)
    rospy.logdebug("Sending goal...")
    moveit_ac.send_goal(moveit_goal)
    rospy.logdebug("Waiting for result...")
    moveit_ac.wait_for_result()
    moveit_result = moveit_ac.get_result()
    
    #rospy.loginfo("Got result:\n" + str(moveit_result)) # Uncomment if you want to see the full result message
    #r = MoveGroupResult()
    if moveit_result != None and moveit_result.error_code.val != 1:
        rospy.logerr("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
        return False
    elif moveit_result != None:
        rospy.logdebug("Goal achieved.")
        return True
    else:
        rospy.logerr("Couldn't get result, something went wrong, the goal probably timed out.")
        return False

if __name__=='__main__':
    send_REEM_to_home_pose()

    
