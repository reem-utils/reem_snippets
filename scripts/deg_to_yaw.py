#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 15 12:42:06 2014

@author: Sam Pfeiffer

Script to get the quaternion to send in a move_base_msgs/MoveBaseGoal message
from a yaw expressed in degrees.

Usually when doing
rosrun actionlib axclient.py /move_base

To fill the orientation field.

"""

from tf.transformations import quaternion_from_euler
import sys
from math import radians

def usage():
    """Print usage and exit"""
    print "Usage:"
    print sys.argv[0] + " yaw_degrees"
    exit(0)

if len(sys.argv) == 1: # no arguments
    usage()
elif len(sys.argv) > 2:
    usage()
else:
    yaw_deg = float(sys.argv[1])
    quat = quaternion_from_euler(0.0, # roll
                                 0.0, # pitch
                                 radians(yaw_deg)) # yaw
    print "    orientation:"
    print "      x: " + str(quat[0])
    print "      y: " + str(quat[1])
    print "      z: " + str(quat[2])
    print "      w: " + str(quat[3])

