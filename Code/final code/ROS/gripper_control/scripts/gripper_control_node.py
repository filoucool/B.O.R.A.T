#!/usr/bin/env python
 
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
 
def handle_set_gripper(req):
    global gripper_status_pub
    gripper_status_pub.publish(req.data)
    return SetBoolResponse(success=True, message="Gripper status set successfully")
 
if __name__ == '__main__':
    rospy.init_node('gripper_control_node')
 
    gripper_status_pub = rospy.Publisher('gripper_status', Bool, queue_size=10)
    set_gripper_srv = rospy.Service('set_gripper', SetBool, handle_set_gripper)
 
    rate = rospy.Rate(10) # 10Hz
    gripper_status = Bool()
 
    while not rospy.is_shutdown():
        gripper_status_pub.publish(gripper_status)
        rate.sleep()
