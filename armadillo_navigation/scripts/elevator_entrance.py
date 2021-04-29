#!/usr/bin/env python 

import rospy
from actionlib_msgs.msg import GoalStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
import tf.transformations
import tf2_ros
from sensor_msgs.msg import LaserScan
import numpy as np
from armadillo_navigation.srv import ser_message, ser_messageResponse

global z, error_max, num
z = 0 
num = -10
error_max = 0

def get_scan(msg):
    global z
    z = np.array(msg.ranges)
         

def elevator_entrance_func(req):
    global z, error_max, num

    rospy.Subscriber('/scan', LaserScan, get_scan)

    input_pose = PoseStamped()
    input_pose.header.stamp = rospy.Time.now()
    input_pose.header.frame_id = 'map'
    input_pose.pose.position.x = 7.0 
    input_pose.pose.position.y = 4.1
    input_pose.pose.position.z = 0
    #input_pose.pose.orientation.x = 
    #input_pose.pose.orientation.y =
    #input_pose.pose.orientation.z =
    #input_pose.pose.orientation.w =

    #converted_pose = tfBuffer.transform(input_pose, 'map').pose

    # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
    #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
    #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
    input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))

    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # moving towards the goal*/
    goal.target_pose = input_pose

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached initial position for elevator entrance")
        # ser_messageResponse(True)
        rospy.wait_for_message("/scan", LaserScan)
        
        # rospy.sleep(0.5)
        temp_z = z
        # flag = True
        while (error_max < 1.5):
            
            error = np.absolute(z - temp_z)
            temp_z = z
            error[~np.isfinite(error)] = 0
            error_max = np.sort(error)[num]
            print ("error max!!: " ,error_max)                
                

            #rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
            rospy.sleep(0.5)
            # if (flag):
            #     flag = False
            #     error_max = 0
            #     rospy.sleep(1)

        if (error_max >= 1.5):
            rospy.loginfo("error_max["+str(num)+"]: "+ str(error_max))
            input_pose = PoseStamped()
            input_pose.header.stamp = rospy.Time.now()
            input_pose.header.frame_id = 'map'
            input_pose.pose.position.x = 6.608 
            input_pose.pose.position.y = 6.670
            input_pose.pose.position.z = 0
            #input_pose.pose.orientation.x = 
            #input_pose.pose.orientation.y =
            #input_pose.pose.orientation.z =
            #input_pose.pose.orientation.w =

            #converted_pose = tfBuffer.transform(input_pose, 'map').pose

            # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
            #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
            #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
            input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))

            #define a client for to send goal requests to the move_base server through a SimpleActionClient
            ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            #wait for the action server to come up
            while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")
            '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
            goal = MoveBaseGoal()
            #set up the frame parameters
            goal.target_pose.header.frame_id = "/map"
            goal.target_pose.header.stamp = rospy.Time.now()
            # moving towards the goal*/
            goal.target_pose = input_pose

            rospy.loginfo("Sending goal location ...")
            ac.send_goal(goal)	
            ac.wait_for_result(rospy.Duration(60))

            if(ac.get_state() ==  GoalStatus.SUCCEEDED):
                rospy.loginfo("You have entered to the elevator")
                # ser_messageResponse(True)

                input_pose = PoseStamped()
                input_pose.header.stamp = rospy.Time.now()
                input_pose.header.frame_id = 'map'
                input_pose.pose.position.x = 6.6 
                input_pose.pose.position.y = 6.4
                input_pose.pose.position.z = 0
                #input_pose.pose.orientation.x = 
                #input_pose.pose.orientation.y =
                #input_pose.pose.orientation.z =
                #input_pose.pose.orientation.w =

                #converted_pose = tfBuffer.transform(input_pose, 'map').pose

                # A fix for the angle transformation, for the case which the "/kinect2/qhd/points" frame is not parallel to the "map" frame Z axis
                #angle_temp = tf.transformations.euler_from_quaternion((converted_pose.orientation.x,converted_pose.orientation.y,converted_pose.orientation.z,converted_pose.orientation.w))
                #converted_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,1.57))
                input_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))

                #define a client for to send goal requests to the move_base server through a SimpleActionClient
                ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
                #wait for the action server to come up
                while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                    rospy.loginfo("Waiting for the move_base action server to come up")
                '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
                    rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
                goal = MoveBaseGoal()
                #set up the frame parameters
                goal.target_pose.header.frame_id = "/map"
                goal.target_pose.header.stamp = rospy.Time.now()
                # moving towards the goal*/
                goal.target_pose = input_pose

                rospy.loginfo("Sending goal location ...")
                ac.send_goal(goal)	
                ac.wait_for_result(rospy.Duration(60))

                if(ac.get_state() ==  GoalStatus.SUCCEEDED):
                    rospy.loginfo("You have entered to the elevator")
                    import shlex
                    from psutil import Popen
                    # start
                    node_process = Popen(shlex.split('rosrun armadillo_navigation_upgrade inside_elevator.py'))
                    # ser_messageResponse(True)
                else:
                    rospy.loginfo("The robot failed to enter to the elevator")
                    # ser_messageResponse(False)

            else:
                rospy.loginfo("The robot failed to enter to the elevator")
                # ser_messageResponse(False)

    else:
        rospy.loginfo("The robot failed to the initial position for elevator entrance")
        # ser_messageResponse(False)

rospy.init_node('elevator_entrance', anonymous=True)
rospy.Service("/elevator_entrance", ser_message, elevator_entrance_func)
rospy.loginfo("elevator entrance service is waiting for request...")

rospy.spin()
