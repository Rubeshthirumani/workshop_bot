#!/usr/bin/env python3
 
import rospy
from obs_service.srv import service_example, service_exampleResponse
 
def turn_on_off(light):
 if light.flag==1:
    print("starting obstacle avoidance service")
    return service_exampleResponse('ON')
 else:
    print("Stopping obstacle avoidance service")
    return service_exampleResponse('OFF')
    
 
rospy.init_node('service_response')
rospy.loginfo("OBSTACLE SERVICE - SERVER STARTED")
 
service=rospy.Service('service_obs',service_example,turn_on_off)
 
rospy.spin()
