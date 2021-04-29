#!/usr/bin/env python
__author__      = "Shashank Shekhar"
__copyright__   = "Copyright (c) 2020, BGU of the Negev, All rights reserved."

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers 

import sys
import copy
import time, datetime
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionGoal

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

global position_x, position_y, position_z, inTime, outTime

position_x = 0.0
position_y = 0.0
position_z = 0.0

def sense_object_callback(coordinates): 
    global position_x, position_y, position_z
    position_x = coordinates.markers[1].pose.pose.position.x
    position_y = coordinates.markers[1].pose.pose.position.y
    position_z = coordinates.markers[1].pose.pose.position.z
    #print("The coordinates received are:", position_x, position_y, position_z)


if __name__ == '__main__':
    global position_x, position_y, position_z

    rospy.init_node('detect_object', anonymous=True)
    print('\nSet-up is done!!\n')
    time.sleep(1)
    inTime = datetime.datetime.now()
    print("in while before")   

    rospy.Subscriber("/detected_objects", AlvarMarkers, sense_object_callback)
    
    pub = rospy.Publisher('/observed_object', String, queue_size=10)
    
    stri = String()
    print("in while")
    while not rospy.is_shutdown():
        outTime = datetime.datetime.now()
        print("in while")
        if( (outTime - inTime).total_seconds() > 10):
            stri = "failed to observe"
            rospy.loginfo(stri)
            pub.publish(stri)
            #rate.sleep()            

        elif(position_x != 0.0 or position_y != 0.0 or position_z != 0.0):
            stri = "coordinates received"
            rospy.loginfo(stri)
            pub.publish(stri)
            #rate.sleep()   
                     
    rospy.wait_for_message('/detected_objects', AlvarMarkers)   

    rospy.spin()
