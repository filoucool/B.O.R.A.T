#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from moveit_msgs.msg import ExecuteTrajectoryActionGoal

class JointStateExecutePublisher:
    def __init__(self):
        self.joint_positions = None

        # Subscribe to the /execute_trajectory/goal topic
        rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, self.execute_trajectory_goal_callback)

        # Publisher to the /joints_state_execute topic
        self.joints_state_execute_pub = rospy.Publisher('/joints_state_execute', Float64MultiArray, queue_size=10)

    def execute_trajectory_goal_callback(self, msg):
        # Extract joint positions from the message
        self.joint_positions = msg.goal.trajectory.joint_trajectory.points[-1].positions

    def publish_joint_state_execute(self):
        # Check if the joint_positions attribute is not None
        if self.joint_positions is not None:
            # Convert joint positions to an array
            joint_states_array = Float64MultiArray()
            joint_states_array.data = list(self.joint_positions)

            # Publish the joint states array to the /joints_state_execute topic
            self.joints_state_execute_pub.publish(joint_states_array)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('joint_state_execute_publisher')

    # Create a JointStateExecutePublisher object
    joint_state_execute_publisher = JointStateExecutePublisher()

    # Set the loop rate
    rate = rospy.Rate(10)

    # Run the node
    while not rospy.is_shutdown():
        joint_state_execute_publisher.publish_joint_state_execute()
        rate.sleep()

