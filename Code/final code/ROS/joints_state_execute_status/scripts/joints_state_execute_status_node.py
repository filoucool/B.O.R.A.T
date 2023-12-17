#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int8

def callback(msg):
    if len(msg.status_list) > 0:
        status = msg.status_list[-1].status
        joints_state_execute_status_pub.publish(status)
    else:
        rospy.logwarn("Empty status_list received")

def main():
    global joints_state_execute_status_pub

    rospy.init_node('joints_state_execute_status_node', anonymous=True)

    rospy.Subscriber('/execute_trajectory/status', GoalStatusArray, callback)

    joints_state_execute_status_pub = rospy.Publisher('/joints_state_execute_status', Int8, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

