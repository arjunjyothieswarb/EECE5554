#!/usr/bin/python3

from mult_driver.srv import convert_to_quaternion, convert_to_quaternionResponse
from geometry_msgs.msg import Quaternion
import rospy
import math

def handle_conversion(req):
    
    quat = Quaternion()

    quat.w = (math.cos(req.euler_angles.x/2) * math.cos(req.euler_angles.y/2) * math.cos(req.euler_angles.z/2)) + (math.sin(req.euler_angles.x/2) * math.sin(req.euler_angles.y/2) * math.sin(req.euler_angles.z/2))
    quat.x = (math.sin(req.euler_angles.x/2) * math.cos(req.euler_angles.y/2) * math.cos(req.euler_angles.z/2)) - (math.cos(req.euler_angles.x/2) * math.sin(req.euler_angles.y/2) * math.sin(req.euler_angles.z/2))
    quat.y = (math.cos(req.euler_angles.x/2) * math.sin(req.euler_angles.y/2) * math.cos(req.euler_angles.z/2)) + (math.sin(req.euler_angles.x/2) * math.cos(req.euler_angles.y/2) * math.sin(req.euler_angles.z/2))
    quat.z = (math.cos(req.euler_angles.x/2) * math.cos(req.euler_angles.y/2) * math.sin(req.euler_angles.z/2)) - (math.sin(req.euler_angles.x/2) * math.sin(req.euler_angles.y/2) * math.cos(req.euler_angles.z/2))

    return(convert_to_quaternionResponse(quat))
    pass

def server_conversion():
    rospy.init_node("quaternion_conversion_server")
    s = rospy.Service("convert_to_quaternion", convert_to_quaternion, handle_conversion)
    print("Service armed")
    rospy.spin()
    pass

if __name__ == '__main__':
    server_conversion()