#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
    deg_data = []
    for rad in data.data:
        deg = rad * 180.0 / 3.141592653589793
        deg_data.append(deg)
    pub.publish(Float64MultiArray(data=deg_data))

if __name__ == '__main__':
    rospy.init_node('joints_state_execute_deg_node')
    pub = rospy.Publisher('/joints_state_execute_deg', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/joints_state_execute', Float64MultiArray, callback)
    rospy.spin()

