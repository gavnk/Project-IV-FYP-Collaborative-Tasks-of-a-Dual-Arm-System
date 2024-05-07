#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    # Your callback logic here
    print("Received joint states:", msg)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('joint_sub', anonymous=True)

    # Set the namespace (replace 'your_namespace' with the actual namespace)
    namespace = '/left_arm'

    # Subscribe to the joint_states topic with the correct namespace
    rospy.Subscriber(namespace + '/joint_states', JointState, joint_states_callback)

    # Spin to keep the node running
    rospy.spin()
