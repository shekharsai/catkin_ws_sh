#!/usr/bin/env python
__author__      = "Shashank Shekhar"
__copyright__   = "Copyright (c) 2020, BGU of the Negev, All rights reserved."

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers 

import sys
import copy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os, time, signal, threading
import subprocess
from subprocess import Popen, PIPE, call
from std_msgs.msg import Float64
from datetime import datetime

global pub
pub = rospy.Publisher('/torso_effort_controller/command', Float64, queue_size=10)

##to perform the finish move in the end
global success_x, success_y, success_z, flag
flag = False
success_x = 0.0
success_y = 0.0
success_z = 0.0

##global variables
global position_x, position_y, position_z
position_x = 0.0
position_y = 0.0
position_z = 0.0
orientation_w =  1.0

def robust_move_group():
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    waypoints = []
    wpose = move_group.get_current_pose().pose
    #pose.position.z -= scale * 0.0  # First move up (z)
    #wpose.position.y += scale * 0.0  # and sideways (y)
    #waypoints.append(copy.deepcopy(wpose))

    print("\n1. error (z):", abs((float)(position_z - wpose.position.z)))    
    #wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
    wpose.position.z += abs((float)(position_z - wpose.position.z)) - 0.001
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:

    #plan = robust_move_group()
    move_group.execute(plan, wait=True) 
    time.sleep(5)

    '''
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    waypoints = []
    wpose = move_group.get_current_pose().pose
    print("\n2. error (y):", abs((float)(position_y - wpose.position.y)))    
    #wpose.position.x += scale * 0.05  # Second move forward/backwards in (x)
    #wpose.position.y += abs((float)(position_y - wpose.position.y)) - 1
    wpose.position.y += abs((float)(position_y - wpose.position.y)) - 0.001
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
    #print("y = ", wpose.position.y)
    # Note: We are just planning, not asking move_group to actually move the robot yet:

    #plan = robust_move_group()
    move_group.execute(plan, wait=True)     
    time.sleep(5)
    '''
    
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    waypoints = []
    wpose = move_group.get_current_pose().pose

    print("\n31. x1 (object):", position_x)
    print("\n32. x2 (gripper):", wpose.position.x)    
    print("\n33. Absolute error (x):", abs((float)(position_x - wpose.position.x)))    
    
    wpose.position.x += abs((float)(position_x - wpose.position.x)) 
    wpose.position.x -= 0.007
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
    #print("y = ", wpose.position.y)
    # Note: We are just planning, not asking move_group to actually move the robot yet:

    #plan = robust_move_group()
    move_group.execute(plan, wait=True)  
    time.sleep(5)
    

    ##remember that the below print statements are required, don't remove those two print statements
    ##also it can open even when last plan was not true, be careful, need to resolve after the deadline
    if (abs((float)(position_x - wpose.position.x)) < 0.05):
        #print('\n\nThe gripper is ', position_y, wpose.position.y, (float)(position_y - wpose.position.y), 'far to the button')
        planning_cobra_center()
        time.sleep(1)        
        #os.system('roslaunch pygazebo_ros_gazebo_elevator elevator_gate_opener.launch') #default floor_num:="1"        
        print("\nSuccessfully pushed the button, (success : True)")

    else:
        planning_cobra_center()
        print("\nCould not push the button: (success : False)", "error is big!")

    #it does not come here
        ##set the torso
  
    #print("after if else")
    time.sleep(2)

def planning_cobra_center():    
    print('Planning to the cobra-center!\n')
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
    
    time.sleep(10)
    proc.terminate()
   
def push_button_go():
    #global pub
    #Beginning#################################################################################################    
    planning_cobra_center()
    ##robust_move_group()    
    count_failure = 0
    ##set the torso
    rate = rospy.Rate(10) # 10hz
    now = datetime.now()  
    
    '''
    ### this code is when there we work with map and need to use the manipulator 
    while not rospy.is_shutdown():
        pub = rospy.Publisher('/torso_effort_controller/command', Float64, queue_size=10)
        msg = Float64()
        msg.data = 0.22
        pub.publish(msg)
        print("pub pub pub !! !! !! !! !! !!")   

        later = datetime.now()
        diff = later - now
        diff_in_seconds = diff.days*24*60*60 + diff.seconds
        if(diff_in_seconds >= 2):
            break 
    
    return
    '''

    ## The code beow is an ideal way to push button, since it moves hand slowly and in pieces. 
    ## With map on, there might be some issues. So directlty calling the waypoint approach may be problematic on some systems
    ## This needs to be fixed for the real robot.
    global success_x, success_y, success_z, flag
 
    # START #################################################################################################    
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    print('\n\nPlan the first move!')
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = orientation_w
    pose_goal.position.x = position_x - 0.4
    pose_goal.position.y = position_y
    pose_goal.position.z = position_z

    print('\n\nCoordinates : ', pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)
    group.set_pose_target(pose_goal)
    group.set_num_planning_attempts = 10
    group.allow_replanning = True
    group.set_planning_time = 20.0
    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    
    if(plan):
        try:
            group.execute(plan, wait=True)            
        except:
            print('\n\nSuccess!')
            if (flag == False):
                #global success_x, success_y, success_z#, flag
                success_x = pose_goal.position.x
                success_y = pose_goal.position.y
                success_z = pose_goal.position.z
                flag = True
    else:
        print('It failed ...')
        count_failure = count_failure + 1
    #END#################################################################################################

    #START#################################################################################################    
    print('\n\nPlan another move ...')
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = orientation_w
    pose_goal.position.x = position_x - 0.3
    pose_goal.position.y = position_y 
    pose_goal.position.z = position_z
    
    print('Coordinates : ', pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)

    group.set_pose_target(pose_goal)
    group.set_num_planning_attempts = 5
    group.allow_replanning = True
    group.set_planning_time = 15.0
    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    
    if(plan):
        try:
            group.execute(plan)            
        except:
            print('\nSuccess again!')
            if (not flag):
                #global success_x, success_y, success_z#, flag
                success_x = pose_goal.position.x 
                success_y = pose_goal.position.y 
                success_z = pose_goal.position.z
                flag = True
    else:
        print('It failed ...')
        count_failure = count_failure + 1
    #END#################################################################################################

    #START#################################################################################################    
    print('\n\nPlan another move ...')
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = orientation_w
    pose_goal.position.x = position_x - 0.2
    pose_goal.position.y = position_y 
    pose_goal.position.z = position_z
    
    print('Coordinates : ', pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)

    group.set_pose_target(pose_goal)
    group.set_num_planning_attempts = 5
    group.allow_replanning = True
    group.set_planning_time = 15.0
    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    
    if(plan):
        try:
            group.execute(plan)            
        except:
            print('\nSuccess again!')
            if (not flag):
                #global success_x, success_y, success_z#, flag
                success_x = pose_goal.position.x 
                success_y = pose_goal.position.y 
                success_z = pose_goal.position.z
                flag = True
    else:
        print('It failed ...')
        count_failure = count_failure + 1
    #END#################################################################################################

    #START#################################################################################################   
    print('\n\nPlan another move ...')
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = orientation_w
    pose_goal.position.x = position_x - 0.1
    pose_goal.position.y = position_y 
    pose_goal.position.z = position_z
    
    print('Coordinates : ', pose_goal.position.x, pose_goal.position.y, pose_goal.position.z)

    group.set_pose_target(pose_goal)
    group.set_num_planning_attempts = 5
    group.allow_replanning = True
    group.set_planning_time = 15.0
    plan = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    
    if(plan):
        try:
            group.execute(plan)            
        except:
            print('\nSuccess again!')
            if (not flag):
                #global success_x, success_y, success_z#, flag
                success_x = pose_goal.position.x
                success_y = pose_goal.position.y 
                success_z = pose_goal.position.z
                flag = True
    else:
        print('It failed ...')
        count_failure = count_failure + 1
    #END#################################################################################################


    #print("how many times failed", count_failure)
    if(count_failure < 4):  
        robust_move_group()
    else:
        print("\nCould not push the button, (success : False)", "the object is too far!!")
	planning_cobra_center()

    #End#################################################################################################       
    
    ###planning_cobra_center()
    '''
    time.sleep(1)
    print('\nPlanning back to cobra-center!\n')
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
    #exit(0)
    '''

def push_button_callback(coordinates):      
    global position_x, position_y, position_z
    position_x = coordinates.markers[1].pose.pose.position.x
    position_y = coordinates.markers[1].pose.pose.position.y
    position_z = coordinates.markers[1].pose.pose.position.z
    #print("poses:", position_x, position_y, position_z)

def set_all_the_hardware(status="pre"):
    import rospy
    from std_msgs.msg import Float64 
    from datetime import datetime
    
    rate = rospy.Rate(10) # 10hz

    ##set the torso
    now = datetime.now()   
    global pub
    while not rospy.is_shutdown():
        #pub = rospy.Publisher('/torso_effort_controller/command', Float64, queue_size=10)
        msg = Float64()
        #print("status", status)
        time.sleep(3)
        if(status == "pre"):
            msg.data = 0.37
        else:
            msg.data = 0.12
        pub.publish(msg)
        rate.sleep()

        later = datetime.now()
        diff = later - now
        diff_in_seconds = diff.days*24*60*60 + diff.seconds
        if(diff_in_seconds >= 2):
            break

    print("\nTorso is set!")    

    '''
    if(status == "pre"):
	print("\nGo to the button!\n")
	time.sleep(1)
	proc = subprocess.Popen(["roslaunch robotican_demos_upgrade adjust_to_push_button.launch"], stdout=PIPE, stderr=PIPE, shell=True, universal_newlines=True) 

	while True:
	    lin = proc.stdout.readline()
	    if "success" in lin and "True" in lin:            
	        break
 	    elif "success" in lin and "False" in lin:     
	        break
	    else:
	        continue  
	time.sleep(1)
	print("Reached close to the button!")
	proc.terminate()    
    '''

    if(status == "pre"):
        #close the Gripper        
        print("\nGripper is closed!")
        #os.system('rosrun robotican_demos_upgrade gripper_close.py')
        pub_gripper = rospy.Publisher('/gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)
        time.sleep(1)
        msg_gripper = GripperCommandActionGoal()
        msg_gripper.goal.command.position=0.02
        #msg_gripper.header.stamp=rospy.Time.now()
        #msg_gripper.header.frame_id=""
        for ii in range(0,3):
            #msg_gripper.goal.command.position=0.02
            #time.sleep(1)
            pub_gripper.publish(msg_gripper)
            time.sleep(1)
        time.sleep(1)                  
        
        print("\nKinect is looking down!")
        pub_joint_trajectory = rospy.Publisher('/pan_tilt_trajectory_controller/command', JointTrajectory, queue_size=10)
        time.sleep(1)
        msg_joint_traj = JointTrajectory()
        msg_joint_traj.header.stamp = rospy.Time.now()
        msg_joint_traj.joint_names.append("head_pan_joint")
        msg_joint_traj.joint_names.append("head_tilt_joint")

        p = JointTrajectoryPoint()
        p.positions.append(0.02)
        p.positions.append(0.55)
        msg_joint_traj.points.append(p)
        
        msg_joint_traj.points[0].time_from_start = rospy.Duration(1.0)
        #msg_joint_traj.points[0].positions = [0.0, 0.55]

        #msg_gripper.header.stamp=rospy.Time.now()
        #msg_gripper.header.frame_id=""
        for ii in range(0,3):
            #msg_gripper.goal.command.position=0.02
            #time.sleep(1)
            pub_joint_trajectory.publish(msg_joint_traj)
            time.sleep(1)
        time.sleep(1)   
        
    return "true"



if __name__ == '__main__':
    
    rospy.init_node('push_button', anonymous=True)
    print("\nSet up the hardware configurations!")
    set_all_the_hardware(status="pre")
    time.sleep(1)
    
    print('\nSet-up is done!!\n')

    rospy.Subscriber("/detected_objects_elevator_button", AlvarMarkers, push_button_callback)
    rospy.wait_for_message('/detected_objects_elevator_button', AlvarMarkers)
    
    push_button_go()
    set_all_the_hardware(status="post")
    rospy.spin()
    
