#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon March 03 17:55:00 2014

@author: Sam Pfeiffer

Send motions to REEM using play_motion.

Usage:
rosrun reem_snippets send_motion <motion_name> <time_to_execute>

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
play_motion_dict = {}
for name in PlayMotionResult.__dict__.keys():
    if not name[:1] == '_':
        code = PlayMotionResult.__dict__[name]
        play_motion_dict[code] = name

def createPlayMotionGoal(motion_name, time):
    """Creates a PlayMotionGoal with the motion_name and time set.
    @arg motion_name string contains the motion name, i.e.: 'arms_t'
    @arg time int time in seconds to accomplish the goal
    @return PlayMotionGoal with the given parameters set"""

    pmg = PlayMotionGoal()
    pmg.motion_name = motion_name
    pmg.reach_time.secs = int(time)
    return pmg

if __name__ == '__main__':
    rospy.init_node('send_play_motion')
    if len(sys.argv) == 3:
        motion = sys.argv[1]
        time = sys.argv[2]
        rospy.loginfo("Will send motion: " + motion + " with timing: " + str(time))
        goal = createPlayMotionGoal(motion, time)
    else:
        rospy.loginfo("Incorrect number of parameters, usage:")
        rospy.loginfo("send_motion.py <motion_name> <time_to_execute>")
        exit(0)
    play_motion_as = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    rospy.loginfo("Connecting to play_motion AS...")
    play_motion_as.wait_for_server()
    rospy.sleep(1.0)
    rospy.loginfo("Connected, sending goal.")
    play_motion_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    play_motion_as.wait_for_result()
    play_motion_result = play_motion_as.get_result()
    rospy.loginfo("Done with result: " + play_motion_dict[play_motion_result.error_code])


    
