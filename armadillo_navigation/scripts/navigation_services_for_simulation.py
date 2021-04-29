#!/usr/bin/env python

import rospy
import time
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from armadillo_navigation.srv import ser_message, ser_messageResponse

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call

rospy.init_node('navigation_services')

def planning_cobra_center():
    #End#################################################################################################
    print('Planning to cobra-center!\n')
    time.sleep(1)
    proc = subprocess.Popen(["roslaunch robotican_demos_upgrade cobra_center.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True)  
    while True:
        lin = proc.stdout.readline()
        if "success" in lin and "True" in lin:            
            break
        elif "success" in lin and "False" in lin:     
            break
        else:
            continue    
    proc.terminate()
    return

def _callback_navigate_corner_area(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(1.350, 4.495, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.552)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the open area")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the open area")
        ser_messageResponse(False)
        time.sleep(1)

def _callback_navigate_open_area(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(5.719, -3.375, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.501)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))
    
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the open area")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the open area")
        ser_messageResponse(False)
        time.sleep(1)


def _callback_navigate_elevator(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(7.650, 3.676, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 1.650)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    time.sleep(4)
    ##repeated just to adjust the location, important for push button
    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(7.191, 4.220, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 1.604)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the elevator")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the elevator")
        ser_messageResponse(False)
        time.sleep(1)

def _callback_navigate_auditorium(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(10.020, -0.656, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -1.519)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the auditorium")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the auditorium")
        ser_messageResponse(False)
        time.sleep(1)

def _callback_navigate_lab_211(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up

    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")

    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(0.356, 0.603, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0.224)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the lab211")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the lab211")
        ser_messageResponse(False)
        time.sleep(1)

def _callback_navigate_outside_lab211(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(-3.133, 3.741, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -3.099)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    time.sleep(2)
    #repeat just for solid execution
    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(-4.352, 4.006, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, -2.884)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the outside of lab211")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the outside of lab211")
        ser_messageResponse(False)
        time.sleep(1)

def _callback_navigate_corridor(req):

    # define a client to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.logwarn("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose.pose.position =  Point(8.659, 3.203, 0)
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0.008)
    goal.target_pose.pose.orientation.x = orientation[0]
    goal.target_pose.pose.orientation.y = orientation[1]
    goal.target_pose.pose.orientation.z = orientation[2]
    goal.target_pose.pose.orientation.w = orientation[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        print("You have reached the corridor")
        ser_messageResponse(True)
        time.sleep(1)
    else:
        print("The robot failed to reach the corridor")
        ser_messageResponse(False)
        time.sleep(1)

#it must be in cobra-center position before starting navigation
planning_cobra_center()

rospy.Service("/elevator_go", ser_message, _callback_navigate_elevator)
rospy.loginfo("navigation service is waiting for request...")

rospy.Service("/auditorium_go", ser_message, _callback_navigate_auditorium)
rospy.loginfo("navigation service is waiting for request...")

rospy.Service("/lab_211_go", ser_message, _callback_navigate_lab_211)
rospy.loginfo("navigation service is waiting for request...")

rospy.Service("/corridor_go", ser_message, _callback_navigate_corridor)
rospy.loginfo("navigation service is waiting for request...")

rospy.Service("/outside_lab_211_go", ser_message, _callback_navigate_outside_lab211)
rospy.loginfo("navigation service is waiting for request...")

rospy.Service("/open_area", ser_message, _callback_navigate_open_area)
rospy.loginfo("navigation service is waiting for request...")

rospy.Service("/corner_area", ser_message, _callback_navigate_corner_area)
rospy.loginfo("navigation service is waiting for request...")



rospy.spin()
