#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun March 03 17:55:00 2014

@author: Sam Pfeiffer

Send motions to REEM using play_motion.

Usage:


Also:

rosrun actionlib axclient.py /play_motion

motion_name: 'arms_t'
reach_time: 
  secs: 3
  nsecs: 0
priority: 0
  
"""

import sys
import actionlib
import rospy


from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult

# Useful dictionary for reading in an human friendly way errors
traj_error_dict = {}
for name in PlayMotionResult.__dict__.keys():
    if not name[:1] == '_':
        code = PlayMotionResult.__dict__[name]
        traj_error_dict[code] = name

def createHandGoal(side, j1, j2, j3):
    """Creates a FollowJointTrajectoryGoal with the values specified in j1, j2 and j3 for the joint positions
    with the hand specified in side
    @arg side string 'right' or 'left'
    @arg j1 float value for joint 'hand_'+side+'_thumb_joint'
    @arg j2 float value for joint 'hand_'+side+'_middle_joint'
    @arg j3 float value for joint 'hand_'+side+'_index_joint'
    @return FollowJointTrajectoryGoal with the specified goal"""
    fjtg = FollowJointTrajectoryGoal()
    fjtg.trajectory.joint_names.append('hand_'+side+'_thumb_joint')
    fjtg.trajectory.joint_names.append('hand_'+side+'_middle_joint')
    fjtg.trajectory.joint_names.append('hand_'+side+'_index_joint')
    point = JointTrajectoryPoint()
    point.positions.append(j1)
    point.positions.append(j2)
    point.positions.append(j3)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.time_from_start = rospy.Duration(4.0)
    fjtg.trajectory.points.append(point)
    for joint in fjtg.trajectory.joint_names: # Specifying high tolerances for the hand as they are slow compared to other hardware
        goal_tol = JointTolerance()
        goal_tol.name = joint
        goal_tol.position = 5.0
        goal_tol.velocity = 5.0
        goal_tol.acceleration = 5.0
        fjtg.goal_tolerance.append(goal_tol)
    fjtg.goal_time_tolerance = rospy.Duration(3)
    fjtg.trajectory.header.stamp = rospy.Time.now()
    return fjtg

if __name__ == '__main__':
    rospy.init_node('move_reem_hand_node')
    if len(sys.argv) == 1:
        rospy.loginfo("Will open right hand.")
        side = "right"
        goal = createHandGoal(side, 0.1, 0.1, 0.1)
    elif len(sys.argv) == 5:
        side = sys.argv[1]
        thumb_joint = float(sys.argv[2])
        middle_joint = float(sys.argv[3])
        index_joint = float(sys.argv[4])
        goal = createHandGoal(side, thumb_joint, middle_joint, index_joint)
        rospy.loginfo("Will move " + side + " hand to poses: " + str(thumb_joint) + " " + str(middle_joint) + " "  + str(index_joint))
    else:
        rospy.loginfo("Incorrect number of parameters, usage:")
        rospy.loginfo("move_reem_hands.py <right/left> <thumb_joint> <middle_joint> <index_joint> ")
        exit(0)
    hand_as = actionlib.SimpleActionClient('/' + side + '_hand_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Connecting to " + side + " hand AS...")
    hand_as.wait_for_server()
    rospy.sleep(1.0)
    rospy.loginfo("Connected, sending goal.")
    hand_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    hand_as.wait_for_result()
    hand_result = hand_as.get_result()
    rospy.loginfo("Done with result: " + traj_error_dict[hand_result.error_code])


    
