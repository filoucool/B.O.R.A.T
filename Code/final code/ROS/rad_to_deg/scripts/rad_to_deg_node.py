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
    rospy.init_node('rad_to_deg_converter')
    pub = rospy.Publisher('/joints_deg_data', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/joints_rad_data', Float64MultiArray, callback)
    rospy.spin()

