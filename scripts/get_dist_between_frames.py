#!/usr/bin/env python
"""
Created on 11/06/14

@author: Sam Pfeiffer
"""

# system stuff
import numpy as np
import sys
# ROS stuff
import rospy
from geometry_msgs.msg import Point, PointStamped
import tf

def usage():
    print "Usage:"
    print sys.argv[0] + " frame_name_1 frame_name_2\n"
    print "For example: " + sys.argv[0] + " hand_left_index_3_link hand_right_index_3_link"
    exit(0)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        usage()
    frame_1 = sys.argv[1]
    frame_2 = sys.argv[2]
    rospy.init_node("dist_bet_frames")
    rospy.sleep(0.3)
    rospy.loginfo("Getting a TransformListener...")
    tf_listener = tf.TransformListener()
    
    # Waiting a moment so the tf_listener catches some transform
    rospy.sleep(0.5)
    # Check if frames exist
    if not tf_listener.frameExists(frame_1):
        print frame_1 + " does not exist on current TF."
        exit(0)
    if not tf_listener.frameExists(frame_2):
        print frame_2 + " does not exist on current TF."
        exit(0)
        
    # Get the point of the tip of the left hand index
    ps = PointStamped()
    ps.point = Point(0.0, 0.0, 0.0)
    ps.header.frame_id = frame_1
    ps.header.stamp = rospy.Time(0) # For getting last transform
    
    # Transform this point to the frame reference of
    # right hand index
    got_transform = False
    while not got_transform:
        try:
            tps = tf_listener.transformPoint(frame_2, ps)
            got_transform = True
        except:
            print "Transformation failed, waiting 0.3 and retrying"
            rospy.sleep(0.3)
            
    # Calculate distances between points
    p1 = np.array(ps.point.__getstate__())
    #print "ps looks like: " + str(ps)
    p2 = np.array(tps.point.__getstate__())
    #print "pts looks like: " + str(tps)
    dist = np.linalg.norm(p1 - p2, ord=3)
    
    print "Distance between the frames " + frame_1 + " and " + frame_2 + " is: " + str(dist)
    