#!/usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

global val_pub, val_msg

def _shutdown():
    global val_pub, val_msg

    val_msg.linear.x = 0
    for ii in xrange(3):
        val_pub.publish(val_msg)

rospy.init_node("inside_elevator", anonymous=True, disable_signals=True)
val_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
node_name_pub = rospy.Publisher("/inside_elevator_node_name", String, queue_size=2)
rospy.on_shutdown(_shutdown)

val_msg = Twist()
val_msg.linear.y = 0
val_msg.linear.z = 0
val_msg.angular.x = 0
val_msg.angular.y = 0
val_msg.angular.z = 0

node_name_msg = String()
node_name_msg.data = rospy.get_name()

rospy.sleep(1)

val_msg.linear.x = -0.01
while not rospy.is_shutdown():
     
    try:
        node_name_pub.publish(node_name_msg)
        val_pub.publish(val_msg)
    except rospy.ROSException as e:
        rospy.logwarn("pub inside elevator node failed: %s"%e)

rospy.signal_shutdown("inside elevator node ended")