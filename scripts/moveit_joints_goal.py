#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 13 19:01:06 2014

@author: Sam Pfeiffer

Snippet of code on how to send a MoveIt! move_group goal to an arm in joint space

Moveit actionserver: /move_group/goal
Type of message: moveit_msgs/MoveGroupGoal

Groups of REEM and their joints:

right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
              'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
              'arm_right_7_joint']
right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                   'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                   'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                   'arm_right_7_joint']
right_arm_torso_grasping = same as right_arm_torso

left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
           'arm_left_7_joint']
left_arm_torso = ['torso_1_joint', 'torso_2_joint',
                   'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                   'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                   'arm_left_7_joint']
left_arm_torso_grasping = same as left_arm_torso

Other groups: 
head = ['head_1_joint', 'head_2_joint']
both_arms = right_arm_torso + left_arm
both_arms_and_head = right_arm_torso + left_arm + head
right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint']
left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']

"""

import rospy
import actionlib
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, MoveItErrorCodes, JointConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def create_move_group_joints_goal(joint_names, joint_values, group="right_arm_torso", plan_only=True):
    """ Creates a move_group goal based on pose.
    @arg joint_names list of strings of the joint names
    @arg joint_values list of digits with the joint values
    @arg group string representing the move_group group to use
    @arg plan_only bool to for only planning or planning and executing
    @return MoveGroupGoal with the desired contents"""
    
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = rospy.Time.now()
    moveit_goal = MoveGroupGoal()
    goal_c = Constraints()
    for name, value in zip(joint_names, joint_values):
        joint_c = JointConstraint()
        joint_c.joint_name = name
        joint_c.position = value
        joint_c.tolerance_above = 0.01
        joint_c.tolerance_below = 0.01
        joint_c.weight = 1.0
        goal_c.joint_constraints.append(joint_c)

    moveit_goal.request.goal_constraints.append(goal_c)
    moveit_goal.request.num_planning_attempts = 5
    moveit_goal.request.allowed_planning_time = 5.0
    moveit_goal.planning_options.plan_only = plan_only
    moveit_goal.planning_options.planning_scene_diff.is_diff = True
    moveit_goal.request.group_name = group
    
    return moveit_goal


if __name__=='__main__':
    rospy.init_node("moveit_snippet")

    rospy.loginfo("Connecting to move_group AS")
    moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    moveit_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.loginfo("Creating goal.")
    # Set a list of joint values for the joints specified
    # joint values for right arm 
    joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                   'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                   'arm_right_7_joint']
    # this is the arm straight down
    joint_list_right_arm_straight_down = [-5.85101288255e-05, -0.00100779755239, 9.26099389043e-05,
                                          0.000257664105577, -1.55489239528e-06, -0.00244347294573,
                                          -2.55958709623e-05]
    # this is the arm in front like if it was going to shake it's hand with someone
    joint_list_right_arm_shake_hand_pose = [0.376906673976, 0.114372113957, -0.198407737748,
                                            1.36616457377, 0.970099953413, 0.108292227188,
                                            -0.822999433641]

    joint_list = joint_list_right_arm_shake_hand_pose
    moveit_goal = create_move_group_joints_goal(joint_names, joint_list, group="right_arm", plan_only=True)
    rospy.loginfo("Sending goal...")
    moveit_ac.send_goal(moveit_goal)
    rospy.loginfo("Waiting for result...")
    moveit_ac.wait_for_result(rospy.Duration(10.0))
    moveit_result = moveit_ac.get_result()
    
    #rospy.loginfo("Got result:\n" + str(moveit_result)) # Uncomment if you want to see the full result message
    #r = MoveGroupResult()
    if moveit_result != None and moveit_result.error_code.val != 1:
        rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
    elif moveit_result != None:
        rospy.loginfo("Goal achieved.")
    else:
        rospy.logerr("Couldn't get result, something went wrong, the goal probably timed out.")
    
