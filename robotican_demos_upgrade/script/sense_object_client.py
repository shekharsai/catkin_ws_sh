#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotican_demos_upgrade.srv import *

def sense_object_client():
    rospy.wait_for_service('sense_object')
    try:
        sense_object_req = rospy.ServiceProxy('sense_object', sense_object) 
        resp1 = sense_object_req("armadillo", "can", "discrete_location")     
        print("Responding to sense unknown!")   
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":    
    resp = sense_object_client()
    print('The response is: ', resp)
    if(resp == "success" or resp == "failure"):
        print("Sensed successfully!!")
        sys.exit()

