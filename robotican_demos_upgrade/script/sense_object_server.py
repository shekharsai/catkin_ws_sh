#!/usr/bin/env python

from __future__ import print_function

import sys
from sys import exit

from std_msgs.msg import String

from robotican_demos_upgrade.srv import *
from robotican_demos_upgrade.srv import sense_objectResponse
import rospy
import time
from datetime import datetime
import signal

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

global inTime, outTime, f_already_launched, globalMessage

globalMessage = "none"

def callback(data):
    #print ("in callback")
    global globalMessage
    globalMessage = data.data

def sense_object_cb(req):
    print("in call back")
    try: 
        global globalMessage, inTime, outTime    
        
        log = ""
        
        p = subprocess.Popen(["roslaunch robotican_demos_upgrade sense_object_launch.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

        print("calling another callback")
        rospy.Subscriber("/observed_object", String, callback) 

        time.sleep(5)
        p.terminate()

        print("globalMessage ", globalMessage)
        if("failed" in globalMessage or "none" in globalMessage):
            message = "false"
            globalMessage = "none"
        if("received" in globalMessage):
            message = "true"
            globalMessage = "none"
        print("returning")
        return sense_objectResponse(message)
    except:
        return sense_objectResponse(message)


def sense_unknown_call():
    global inTime, outTime
    rospy.init_node('sense_object_server')
    inTime = datetime.now()
    s = rospy.Service('sense_object', sense_object, sense_object_cb)
    print("Ready to sense the object!")
    rospy.spin()
    
if __name__ == "__main__":
    sense_unknown_call()
