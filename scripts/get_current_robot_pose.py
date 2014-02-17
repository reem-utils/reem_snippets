#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 10 11:16:00 2014

@author: Sam Pfeiffer

Snippet of code on how to get the current robot position in map


"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees


def callback_pose(data):
    """Callback for the topic subscriber.
       Prints the current received data on the topic."""
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                             data.pose.pose.orientation.y,
                                             data.pose.pose.orientation.z,
                                             data.pose.pose.orientation.w])
    rospy.loginfo("Current robot pose: x=" + str(x) + " y=" + str(y) + " yaw=" + str(degrees(yaw)) + "º rads=" + str(yaw))

if __name__=='__main__':
    rospy.init_node("navigation_snippet")
    
    # Read the current pose topic
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)
    
    rospy.spin()

    
    
