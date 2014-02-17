#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 16 22:46:20 2014

@author: Sam Pfeiffer

Send goals to the TTS with what was recognized by the ASR.

We expect that the ASR already initialized.

The robot will say "Did you say <text recognized> ?".
  
"""

import actionlib
import rospy
from pal_interaction_msgs.msg import SoundAction, SoundGoal
from pal_interaction_msgs.msg import asrresult


class TTS_ASR():
    def __init__(self):
        self.tts_as = actionlib.SimpleActionClient('/sound', SoundAction)
        rospy.loginfo("Connecting to TTS AS...")
        self.tts_as.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected.")
        # Read the current asrresult
        rospy.loginfo("Subscribing to /usersaid...")
        sub_usersaid = rospy.Subscriber("/usersaid", asrresult, self.callback_asr)
        self.usersaid = None
        while self.usersaid == None:
            rospy.sleep(0.1)
        rospy.loginfo("Done, ready!")
            

    def createTTSGoal(self, text, lang_id='', wait_before_speaking=rospy.Duration(0.0)):
        """Creates a 
        @arg j1 float value for head_1_joint
        @returns FollowJointTrajectoryGoal with the specified goal"""
        sound_goal = SoundGoal()
        sound_goal.text=text
        sound_goal.lang_id=lang_id
        sound_goal.wait_before_speaking=wait_before_speaking
        return sound_goal
    
    def callback_asr(self, data):
        """Callback for the topic subscriber.
           Prints the current received data on the topic."""
        self.usersaid = data
        goal = self.createTTSGoal("Did you say " + str(data.text) + "?")
        self.tts_as.send_goal(goal)
        rospy.loginfo("Goal sent, waiting...")
        self.tts_as.wait_for_result(rospy.Duration(10))

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('make_reem_say_what_it_heard')
    
    node = TTS_ASR()







    
