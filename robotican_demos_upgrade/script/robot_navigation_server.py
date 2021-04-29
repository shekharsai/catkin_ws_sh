#!/usr/bin/env python

from __future__ import print_function

import sys
from sys import exit

from robotican_demos_upgrade.srv import *
from armadillo_navigation.srv import robot_navigationResponse
import rospy
import time
from datetime import datetime
import signal

import os, time, signal, threading
import subprocess, shlex
from subprocess import Popen, PIPE, call

global inTime, outTime, f_already_launched 

f_already_launched = False


def handle_stdout(proc_stream, my_buffer, echo_streams=True, log_file=None):
    """A little inline function to handle the stdout business. """
    # fcntl makes readline non-blocking so it raises an IOError when empty
    try:
        for s in iter(proc_stream.readline, ''):   # replace '' with b'' for Python 3
            #print("in", s)
            time.sleep(1)
            my_buffer.append(s)

            if echo_streams:
                sys.stdout.write(s)

            if log_file:
                log_file.write(s)
    except IOError:
        pass
    #print("exit")

def robot_navigation_cb(req):
    try:  
        message = ""

        print("action :", req.nav_name)
        
        ##these are manually executing launches, need to figure out a decent way to do it, but later
        if(req.nav_name == "elevator"):           
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_elevator.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True) 
        
        elif(req.nav_name == "enter_elevator"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_enter_elevator.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

        elif(req.nav_name == "corridor"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_corridor.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

        elif(req.nav_name == "auditorium"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_auditorium.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

        elif(req.nav_name == "lab211"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_lab211.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)
        
        elif(req.nav_name == "outside_lab211"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_outside_lab211.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)
        
        elif(req.nav_name == "open_area"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_open_area.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)
        
        elif(req.nav_name == "corner_area"):            
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_corner_area.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)
        
        ## By default it will be near the elevator
        else:           
            p = subprocess.Popen(["roslaunch /home/lab16/catkin_ws_elevator/src/robotican_demos_upgrade/launch/robot_navigation_elevator.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)

        iwlist_output = p.communicate()[0].decode('utf-8')
        print("ss", iwlist_output)
        time.sleep(2)

        import fcntl
        proc_stdout = p.stdout
        fl = fcntl.fcntl(proc_stdout, fcntl.F_GETFL)
        fcntl.fcntl(proc_stdout, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        stdout_parts = []
        index = 0
        while p.poll() is None:
            handle_stdout(proc_stdout, stdout_parts, echo_streams=True)
            # ...Check for other things here...
            # For example, check a multiprocessor.Value('b') to proc.kill()
            print("ss ", proc_stdout)
            time.sleep(0.1)
            index += 1
            if(index > 700):
                message = "true"
                break
        

        

        #print("Output of the last service:\n\n", log)
        print ("\n\nTerminating the navigation node!\n")              

        #p.terminate()
        return robot_navigationResponse(message)        
    except:
        #p.terminate()
        return robot_navigationResponse(message)


def push_button_call():
    rospy.init_node('robot_navigation_server')
    s = rospy.Service('robot_navigation', robot_navigation, robot_navigation_cb)
    print("Ready to navigation from one location to another!")
    rospy.spin()
    
if __name__ == "__main__":
    push_button_call()
