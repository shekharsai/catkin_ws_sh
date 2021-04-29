#!/usr/bin/env python

import rospy 
import sys 
#import roslib
import pygazebo
import trollius
from trollius import From
#from pygazebo_ros_gazebo_elevator.srv import ser_message # ,ser_messageResponse

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
def setObject(floor):
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/elevator',
                          'gazebo.msgs.GzString'))

    #pub = rospy.Publisher('/gazebo/default/elevator', GzString, queue_size=10)
    #rospy.init_node('setObject', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    msg = GzString() 
    msg.data = str(floor) 
    #pub.publish(msg)
    # c = 0
    # while True:
        # print c
        # c = c+1
    for ii in xrange (3):
        yield From(publisher.publish(msg))
        yield From(trollius.sleep(1.0))

floor = rospy.myargv(argv=sys.argv)

if len(floor) == 2:
    rospy.loginfo("Opening the elevator gate")
else:
    rospy.logwarn("floor_num should be 0 or 1, but it contain: "+ str(len(floor)))

loop = trollius.get_event_loop()
loop.run_until_complete(setObject(floor[1]))

    #rate.sleep()
    #print  "here"

#if __name__ == '__main__':
   # try:
   #      setObject()
  #  except rospy.ROSInterruptException:
    #     pass
