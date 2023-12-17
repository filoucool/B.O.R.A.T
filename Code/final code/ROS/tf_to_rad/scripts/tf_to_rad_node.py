#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointPositionConverter:
    def __init__(self):
        rospy.init_node('joint_position_converter', anonymous=True)
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.joints_rad_data_pub = rospy.Publisher('/joints_rad_data', Float64MultiArray, queue_size=10)

    def joint_states_callback(self, msg):
        joint_positions = Float64MultiArray()
        joint_positions.data = msg.position
        self.joints_rad_data_pub.publish(joint_positions)

if __name__ == '__main__':
    joint_position_converter = JointPositionConverter()
    rospy.spin()

