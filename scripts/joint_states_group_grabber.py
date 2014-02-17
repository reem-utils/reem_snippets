#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 13 19:06:55 2014

@author: Sam Pfeiffer

Subscribe to joint states and get the joint positions for a group printed nicely in screen or
import from this file to get joint names and values of a group using getNamesAndMsgList
for any tool you need
"""
import sys

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

groups = ["all_joints", "left_arm", "right_arm", "left_arm_torso", "right_arm_torso", "torso", "head", "right_hand", "left_hand", "right_hand_all", "left_hand_all"]
shortnamegroups = ["a", "la", "ra", "lat", "rat", "t", "h", "rh", "lh", "rha", "lha"]


class jointStateGrabber():
    """This class subscribes to the /joint_states topic of REEM
    and you can ask it for the current status of a group and get it 
    outputted in some useful format, i.e.: print on screen,
    play_motion format, old xml format... """

    def __init__(self):
        self.current_joint_states = None
        self.all_joints = ['torso_1_joint', 'torso_2_joint',
                           'head_1_joint', 'head_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint', 
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint',
                           'hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint',
                           'hand_right_index_joint', 'hand_right_middle_1_joint', 'hand_right_middle_2_joint',
                           'hand_right_middle_3_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint',
                           'hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint',
                           'hand_left_index_joint', 'hand_left_middle_1_joint', 'hand_left_middle_2_joint',
                           'hand_left_middle_3_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.head = ['head_1_joint', 'head_2_joint']
        self.torso = ['torso_1_joint', 'torso_2_joint']
        self.right_hand_all = ['hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint',
                               'hand_right_index_joint', 'hand_right_middle_1_joint', 'hand_right_middle_2_joint',
                               'hand_right_middle_3_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint']
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] # Only the actuated
        self.left_hand_all = ['hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint',
                          'hand_left_index_joint', 'hand_left_middle_1_joint', 'hand_left_middle_2_joint',
                          'hand_left_middle_3_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
        self.left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint'] # Only the actuated
        
        self.ids_list = []

        self.subs = rospy.Subscriber('/joint_states', JointState, self.getJointStates)
        
        # getting first message to correctly find joints
        while self.current_joint_states == None:
            rospy.sleep(0.1)
        rospy.loginfo("Node initialized. Ready to grab joint states.")
        

    def getJointStates(self, data):
        #rospy.loginfo("Received from topic data!")
        self.current_joint_states = data
        
    def createGoalFromCurrentJointStateForArm(self, group='right_arm_torso'):
        """ Get the joints for the specified group and create a FollowJointTrajectoryGoal
        with these joints and values for the joints """
        names, values = self.getNamesAndMsgList(group=group)
    
        fjtg = FollowJointTrajectoryGoal()
        fjtg.trajectory.joint_names.extend(names)
        jtp = JointTrajectoryPoint(positions=values, velocities=len(values) * [0.0], time_from_start=0)
        fjtg.trajectory.points.append(jtp)
        
        rospy.loginfo("follow joint trajectory goal:\n" + str(fjtg))
        
        return fjtg
        
        
    def getNamesAndMsgList(self, group='right_arm_torso'):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """
        
        list_to_iterate = getattr(self, group)        
        curr_j_s = self.current_joint_states
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))
    
        return list_to_iterate, msg_list

    def printNamesAndValues(self, group='right_arm_torso'):
        """Given a group, print in screen in a pretty way joint names and it's values"""
        names, values = self.getNamesAndMsgList(group=group)
        print "Name =  Joint Value"
        print "================="
        for nam, val in zip(names, values):
            print nam + " = " + str(val)
            
    def printOnlyValues(self, group):
        names, values = self.getNamesAndMsgList(group=group)
        print group + " = " + str(names)
        print "[ ",
        for val in values:
            if val == values[-1]: # if it's the last one dont put comma and add an enter
                print str(val) + " ]"
            else:
                print str(val)+",", 


def usage(program_name):
    """ print usage """
    print "Usage:"
    print program_name
    print "Print all joint names and it's values.\n"
    print program_name + " -i"
    print "Interactive mode, write a group name and it will be printed.\n"
    print program_name + " <group>"
    print "Will print the group joint names and its joint values."
    print "Available groups are: " + str(groups)
    print "Or their shortname version: " + str(shortnamegroups)

def shortToLargeName(shortname):
    """ Translate shortname of group to long name of group"""
    try:
        longname = groups[shortnamegroups.index(shortname)]
    except ValueError:
        print "\"" + shortname + "\" not a valid short name."
        print "Valid short names:"
        print shortnamegroups
        exit(0)
    return longname

def isGroupName(supposed_group_name):
    """ Check if a given group string is a group or shortname for a group"""
    if isShortGroupName(supposed_group_name):
        return True
    elif not isLongGroupName(supposed_group_name):
        return False
    return True

def isShortGroupName(group_name):
    """Check if a given group is a shortname for a group"""
    try:
        idx = shortnamegroups.index(group_name)
    except ValueError:
        return False
    return True

def isLongGroupName(group_name):
    """Check if a given group is a name for a group"""
    try:
        idx = groups.index(group_name)
    except ValueError:
        return False
    return True

def getGroupNameIfExists(group_name):
    """Returns the group name if it exists None otherwise"""
    if isGroupName(group_name): # if a valid name is given for a group, print that group
        if isShortGroupName(group_name):
            return shortToLargeName(group_name)
        else:
            return group_name
    else:
        return None
    

if __name__ == '__main__':
    rospy.init_node('joint_state_grabber')
    node = jointStateGrabber()
    if len(sys.argv) > 3:
        print "Error, too many arguments"
        usage(sys.argv[0])
        exit()
    elif len(sys.argv) == 1: # default to print all joints
        group_to_print = "all_joints"
    elif len(sys.argv) == 2: # different options...
        group_to_print = getGroupNameIfExists(sys.argv[1])
        if group_to_print == None and sys.argv[1] != "-i":
            print "Not a valid group name."
            usage(sys.argv[0])
            exit(0)
        elif sys.argv[1] == "-i":
            print "Interactive mode! Write a group name (or short group name) and it's values will be printed. (Write exit to exit)."
            print groups
            print shortnamegroups
            while True:
                input = raw_input("> ")
                group_to_print = getGroupNameIfExists(input)
                if input == "exit":
                    exit(0)
                if group_to_print == None:
                    print "Not a valid group name!"
                    print "Short names: " + str(shortnamegroups)
                else:
                    node.printNamesAndValues(group_to_print)
                    node.printOnlyValues(group_to_print)
              
    node.printNamesAndValues(group_to_print)
