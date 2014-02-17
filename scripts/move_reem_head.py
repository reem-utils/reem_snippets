#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 12 18:19:20 2014

@author: Sam Pfeiffer

Send goals to the motors of the head.

Usage:
move_reem_head.py <head_1_joint> <head_2_joint>
           pan (+ left - right)  tilt (+ down - up)
or (without parameters)
move_reem_head.py
Which will move the head down.

Also:

rosrun actionlib axclient.py /head_controller/follow_joint_trajectory

trajectory: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs: 0
    frame_id: ''
  joint_names: ['head_1_joint','head_2_joint']
  points:
      -
        positions: [0.0, 0.0]
        velocities: [0.0, 0.0]
        accelerations: []
        time_from_start: 
          secs: 2
          nsecs: 0
path_tolerance: []
goal_tolerance: []
goal_time_tolerance: 
  secs: 0
  nsecs: 0
  
  
"""

import sys
import actionlib
import rospy


from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

# Useful dictionary for reading in an human friendly way errors
traj_error_dict = {}
for name in FollowJointTrajectoryResult.__dict__.keys():
    if not name[:1] == '_':
        code = FollowJointTrajectoryResult.__dict__[name]
        traj_error_dict[code] = name

def createHeadGoal(j1, j2):
    """Creates a FollowJointTrajectoryGoal with the values specified in j1 and j2 for the joint positions
    @arg j1 float value for head_1_joint
    @arg j2 float value for head_2_joint
    @returns FollowJointTrajectoryGoal with the specified goal"""
    fjtg = FollowJointTrajectoryGoal()
    fjtg.trajectory.joint_names.append('head_1_joint')
    fjtg.trajectory.joint_names.append('head_2_joint')
    point = JointTrajectoryPoint()
    point.positions.append(j1)
    point.positions.append(j2)
    point.velocities.append(0.0)
    point.velocities.append(0.0)
    point.time_from_start = rospy.Duration(4.0)
    for joint in fjtg.trajectory.joint_names: # Specifying high tolerances for the hand as they are slow compared to other hardware
        goal_tol = JointTolerance()
        goal_tol.name = joint
        goal_tol.position = 5.0
        goal_tol.velocity = 5.0
        goal_tol.acceleration = 5.0
        fjtg.goal_tolerance.append(goal_tol)
    fjtg.goal_time_tolerance = rospy.Duration(3)
    
    fjtg.trajectory.points.append(point)
    fjtg.trajectory.header.stamp = rospy.Time.now()
    return fjtg

if __name__ == '__main__':
    rospy.init_node('move_reem_head_node')
    if len(sys.argv) == 1:
        rospy.loginfo("Will move head down.")
        goal = createHeadGoal(0.0, 1.0)
    elif len(sys.argv) == 3:
        head_1_joint = float(sys.argv[1])
        head_2_joint = float(sys.argv[2])
        goal = createHeadGoal(head_1_joint, head_2_joint)
        rospy.loginfo("Will move head to poses: " + str(head_1_joint) + " " + str(head_2_joint))
    else:
        rospy.loginfo("Incorrect number of parameters, usage:")
        rospy.loginfo("move_reem_head.py <head_1_joint> <head_2_joint>")
        rospy.loginfo("           pan (+ left - right)  tilt (+ down - up)")
        exit(0)
    head_as = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Connecting to head AS...")
    head_as.wait_for_server()
    rospy.sleep(1.0)
    rospy.loginfo("Connected, sending goal.")
    head_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    head_as.wait_for_result()
    #head_result = head_as.get_result()
    #rospy.loginfo("Done with result: " + traj_error_dict[head_result.error_code.val])


    
