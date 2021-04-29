#!/usr/bin/env python

from __future__ import print_function

import sys
from sys import exit

from robotican_demos_upgrade.srv import *
from robotican_demos_upgrade.srv import push_buttonResponse
import rospy
import time
from datetime import datetime
import signal

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

global inTime, outTime, f_already_launched 

f_already_launched = False

def push_elevator_button_cb(req):
    try:        
        '''
        import os
        global manual_shutdown
        os.system('roslaunch robotican_demos_upgrade push_button_launch.launch')   
        '''   

        global f_already_launched, inTime, outTime
        inTime = datetime.now()

        ##these are manually executing launches, need to figure out a decent way to do it, but later
        if( not f_already_launched ): 
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/push_button_launch.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True) 
            f_already_launched = True
        else:
            print('\n\nServer is not running for the first time!\n')
            time.sleep(2)
            p = subprocess.Popen(["rosrun robotican_demos_upgrade push_button.py"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True) 

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

        return push_buttonResponse(message)
    except:
        return push_buttonResponse(message)


def push_button_call():
    rospy.init_node('push_button_server')
    s = rospy.Service('push_button', push_button, push_elevator_button_cb)
    print("Ready to push the elevator button!")
    rospy.spin()
    
if __name__ == "__main__":
    push_button_call()
