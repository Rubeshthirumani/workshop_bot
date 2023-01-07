#!/usr/bin/env python3

#importing libraries and msgs
 
import rospy
 
from obs_service.srv import service_example
 
import sys


  
rospy.init_node('client_call')
 
#wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('service_obs')
 
#setup a local proxy for the service
srv=rospy.ServiceProxy('service_obs',service_example)
 
#use the service and send it a value. In this case, I can send 1 or 0
result=srv(1)
 
#print the result from the service
print (result)