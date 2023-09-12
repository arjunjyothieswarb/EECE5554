#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def Callback(data):
    print("Was it " + data.data + " ?")
    pass

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, Callback)

    rospy.spin()
    pass

if __name__ == '__main__':
    listener()
    pass