#!/usr/bin/env python

from __future__ import print_function

import sys
from sys import exit

from robotican_demos_upgrade.srv import *
from robotican_demos_upgrade.srv import place_unknownResponse
import rospy
import time
from datetime import datetime
import signal

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

global inTime, outTime, f_already_launched 

f_already_launched = False

def place_unknown_cb(req):

    try: 
        global f_already_launched  
        global inTime, outTime
        inTime = datetime.now()

        ##these are manually executing launches, need to figure out a decent way to do it, but later
        p = subprocess.Popen(["roslaunch robotican_demos_upgrade place_unknown_launch.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)
   
        log = ""
        message = ""
        while True:
            o = p.stdout.readline()
            log += o
            if "success" in o and "True" in o:
                message = "true"
                break
            elif "success" in o and "False" in o:
                message = "false"
                break
            else:
                continue
        
        print("Output of the last servise:\n\n", log)
        print ("\n\nTerminating the push button node!\n")
        

        return place_unknownResponse(message)
    except:
        return place_unknownResponse(message)


def pick_unknown_call():
    rospy.init_node('place_unknown_server')
    s = rospy.Service('place_unknown', place_unknown, place_unknown_cb)
    print("Ready to place the object!")
    rospy.spin()
    
if __name__ == "__main__":
    pick_unknown_call()
