#!/usr/bin/env python

import rospy
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees, cos,sin
from actionlib_msgs.msg import *

from aos_project.srv import ser_message, ser_messageResponse
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from tf import transformations

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
#from pal_interaction_msgs.msg import TtsAction, TtsGoal
from move_base_msgs.msg import MoveBaseActionResult

#from object_recognition.voice_recognition import SpeechDetector

xCafeMiddle = -6.20
yCafeMiddle = -3.90
euler_z = 3.14
xHumanPoint = -0.124812
yHumanPoint = 3.345588
euler_HumanPoint = 1.929115
ZeroPoint = 0

p = rospy.Publisher('/plp/trigger',String, queue_size=10)
stat_pub = rospy.Publisher("/move_base/result", MoveBaseActionResult, queue_size=10)

def speech(req):
    global client
    goal = TtsGoal()
    goal.rawtext.text = str("please take the cube")
    goal.rawtext.lang_id = "EN"
    goal.wait_before_speaking = float(0)
    client.send_goal(goal, feedback_cb = "")
    client.wait_for_result()
    return ser_messageResponse(True)

def ask_robot(req):
    rospy.loginfo("starting to listen:")
    #sd = SpeechDetector()
    #sd.run(query_callback)

#Parse query, and retrieve recognized object coordinates.
def query_callback(query, ignore_params=None):
    query = query.replace('cap', 'cup')
    query = query.replace('battle', 'bottle')
    print(query)


def moveToGoal(xGoal,yGoal,euler_z):

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
    qaut = transformations.quaternion_from_euler(0,0,euler_z)
    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = qaut[0]
    goal.target_pose.pose.orientation.y = qaut[1]
    goal.target_pose.pose.orientation.z = qaut[2]
    goal.target_pose.pose.orientation.w = qaut[3]

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)	
    ac.wait_for_result(rospy.Duration(60))

    stat = MoveBaseActionResult()
    stat.status.status = 0
    stat_pub.publish(stat)
    
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return ser_messageResponse(True)

    else:
            rospy.loginfo("The robot failed to reach the destination")
            return ser_messageResponse(False)

def give_human(req):
    p.publish("start give")
    data = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState, 5)
    positions_arm = data.actual.positions

    global client
    goal = TtsGoal()
    goal.rawtext.text = str("please take the cup")
    goal.rawtext.lang_id = "EN"
    goal.wait_before_speaking = float(0)

    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
    waypoints = []

    wpose = group.get_current_pose().pose
    scaley = -0.2-wpose.position.y
    # wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scaley * 1.0  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    scalex = 0.7-wpose.position.x
    print(wpose.position.x)
    wpose.position.x += scalex * 1.0  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    scalez = 1.0-wpose.position.z
    wpose.position.z += scalez * 1.0  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                            waypoints,   # waypoints to follow
                                            0.005,        # eef_step
                                            0.0)         # jump_threshold


    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    #     group.execute(plan, wait=True)
    #     group.execute(plan, wait=True)
    client.send_goal(goal, feedback_cb = "")
    client.wait_for_result()
    p.publish("finish give")
    return ser_messageResponse(True)

def go_to_zero(req):
    p.publish("start move_to_zero_point")
    data = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState, 5)
    positions_arm = data.actual.positions
    if positions_arm[0] > 2 and positions_arm[4] - 0.2 > -1.8 :
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        joint_goal = group.get_current_joint_values()
        joint_goal[4] = positions_arm[4] - 0.2
        
        group.go(joint_goal, wait=True)
        group.stop()
    elif positions_arm[4] + 0.2 < 1.8:
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        joint_goal = group.get_current_joint_values()
        joint_goal[4] = positions_arm[4] + 0.2
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
    waypoints = []

    wpose = group.get_current_pose().pose
    scale = 0.0-wpose.position.y
    # wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 1  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold


    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    group.execute(plan, wait=True)
    return moveToGoal(ZeroPoint, ZeroPoint, ZeroPoint)

def go_to_human(req):
    p.publish("start move_to_human")
    return moveToGoal(xHumanPoint, yHumanPoint, euler_HumanPoint)

def go_to_room(req):
    p.publish("start move_to_room")
    return moveToGoal(xCafeMiddle, yCafeMiddle, euler_z)

def find_gui(req):
    t = String()
    t = "t"
    p.publish(t)
    rospy.sleep(1)
    return True

def run_services():
    #global client
    rospy.init_node("services")
    #client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
    rospy.loginfo("Waiting for server")
    #client.wait_for_server()
    rospy.loginfo("Reached server")
    #rospy.Service("/navigate_room", ser_message, go_to_room)
    #rospy.Service("/navigate_zero", ser_message, go_to_zero)
    #rospy.Service("/navigate_human", ser_message, go_to_human)
    rospy.Service("/give_gui", ser_message, give_human)
    #rospy.Service("/find_gui", ser_message, find_gui)
    #rospy.Service("/speech_gui", ser_message, speech)
    #rospy.Service("/ask_robot", ser_message, ask_robot)

if __name__ == "__main__":
    run_services()    
    rospy.spin()
