#!/usr/bin/env python

from __future__ import print_function

import sys
from sys import exit

from robotican_demos_upgrade.srv import *
from robotican_demos_upgrade.srv import pick_unknownResponse
import rospy
import time
from datetime import datetime
import signal

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

global f_already_launched 

f_already_launched = False

def pick_unknown_cb(req):

    try: 
        inTime = datetime.now()

        message = ""
        log = ""
        
        proc = subprocess.Popen(["roslaunch robotican_demos_upgrade pick_unknown_launch.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)  
        while True:
            lin = proc.stdout.readline()
            #print (lin)
            log += lin
            if "success" in lin and "True" in lin:
                message = "true"
                break
            elif "success" in lin and "False" in lin:
                message = "false" 
                break
            elif "object" in lin and "not" in lin and "frame" in lin:
                message = "invalid"
                break
            else:
                continue        

        proc.terminate()        
        time.sleep(3)
        print("Output of the last service:\n\n", log)
        print ("\n\nTerminating the pick object node!\n")

        return pick_unknownResponse(message)
    except:
        return pick_unknownResponse(message)


def pick_unknown_call():
    rospy.init_node('pick_unknown_server')
    s = rospy.Service('pick_unknown', pick_unknown, pick_unknown_cb)
    print("Ready to pick the object!")
    rospy.spin()
    
if __name__ == "__main__":
    pick_unknown_call()
