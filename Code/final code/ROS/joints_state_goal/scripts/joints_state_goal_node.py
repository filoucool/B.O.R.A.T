#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import MoveGroupActionGoal

class JointStateGoalPublisher:
    def __init__(self):
        self.joint_positions = None

        # Subscribe to the /move_group/goal topic
        rospy.Subscriber('/move_group/goal', MoveGroupActionGoal, self.move_group_goal_callback)

        # Publisher to the /joints_state_goal topic
        self.joints_state_goal_pub = rospy.Publisher('/joints_state_goal', Float64MultiArray, queue_size=10)

    def move_group_goal_callback(self, msg):
        # Extract joint positions from the message
        self.joint_positions = msg.goal.request.goal_constraints[0].joint_constraints

    def publish_joint_state_goal(self):
        # Check if the joint_positions attribute is not None
        if self.joint_positions is not None:
            # Combine joint positions into an array
            joint_states_array = Float64MultiArray()
            joint_states_array.data = [joint.position for joint in self.joint_positions]

            # Publish the joint states array to the /joints_state_goal topic
            self.joints_state_goal_pub.publish(joint_states_array)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('joint_state_goal_publisher')

    # Create a JointStateGoalPublisher object
    joint_state_goal_publisher = JointStateGoalPublisher()

    # Set the loop rate
    rate = rospy.Rate(10)

    # Run the node
    while not rospy.is_shutdown():
        joint_state_goal_publisher.publish_joint_state_goal()
        rate.sleep()

