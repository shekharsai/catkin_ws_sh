#!/usr/bin/env python

import rospy  
import roslib
import pygazebo
import trollius
from trollius import From

#import rospkg
#rospack = rospkg.RosPack()
#PATH_TO_armadillo_navigation_upgrade = rospack.get_path('armadillo_navigation_upgrade')
#import sys
#sys.path.append(PATH_TO_armadillo_navigation_upgrade)

#from msg import gz_string_pb2.GzString
from msg.gz_string_pb2 import GzString

#from std_msgs.msg import String
#from gazebo_msgs.msg import GzString
#from pygazebo.msg.gz_string_pb2 import GzString

@trollius.coroutine
def setObject():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/elevator', 'gazebo.msgs.GzString'))

    #pub = rospy.Publisher('/gazebo/default/elevator', GzString, queue_size=10)
    #rospy.init_node('setObject', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    msg = GzString() 
    msg.data = "1" 
    #pub.publish(msg)
    while True:
        yield From(publisher.publish(msg))
        yield From(trollius.sleep(1.0))

loop = trollius.get_event_loop()
loop.run_until_complete(setObject())

    #rate.sleep()
    #print  "here"

#if __name__ == '__main__':
   # try:
   #      setObject()
  #  except rospy.ROSInterruptException:
    #     pass
