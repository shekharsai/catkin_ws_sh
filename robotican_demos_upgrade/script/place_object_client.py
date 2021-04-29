#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotican_demos_upgrade.srv import *

def place_unknown_client():
    rospy.wait_for_service('place_unknown')
    try:
        place_unknown_req = rospy.ServiceProxy('place_unknown', place_unknown) 
        resp1 = place_unknown_req("armadillo", "can", "table", "discrete_location")     
        print("Responding to place unknown!")   
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":    
    resp = place_unknown_client()
    print('The response is: ', resp)
    if(resp == "success" or resp == "failure"):
        print("Placed successfully!!")
        sys.exit()

