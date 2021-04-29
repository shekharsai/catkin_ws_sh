#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotican_demos_upgrade.srv import *

def pick_unknown_client():
    rospy.wait_for_service('pick_unknown')
    try:
        pick_unknown_req = rospy.ServiceProxy('pick_unknown', pick_unknown) 
        resp1 = pick_unknown_req("armadillo", "can", "discrete_location")     
        print("Responding to pick unknown!")   
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":    
    resp = pick_unknown_client()
    print('The response is: ', resp)
    if(resp == "success" or resp == "failure"):
        print("Picked successfully!!")
        sys.exit()

