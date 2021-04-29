#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotican_demos_upgrade.srv import *

def push_button_client():
    rospy.wait_for_service('robot_navigation')
    try:
        robot_navigation_req = rospy.ServiceProxy('robot_navigation', robot_navigation) 
        resp1 = robot_navigation_req("corridor", "armadillo", "location1", "location2", "floor")     
        print("responding!")   
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":    
    resp = push_button_client()
    print('The response is: ', resp)
    if(resp == "success" or resp == "failure"):
        print("pushed the button successfully!!")
        sys.exit()


'''
#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotican_demos_upgrade.srv import *

def push_button():
    rospy.wait_for_service('push_button')
    try:
        push_button_req = rospy.ServiceProxy('push_button', push_button) 
        resp1.response = push_button_req("push_button", "armadillo", "elevator", "floor", "button")        
        print(resp1.response)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":    
    push_button()
    #if(resp == "success" or resp == "failure"):
    #    sys.exit() 
    
'''