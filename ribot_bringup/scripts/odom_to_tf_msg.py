#!/usr/bin/env python3

"""
Script which converts a nav_msgs/Odometry to a tf2_msgs/TFMessage message.
"""

import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

def odom_to_tf(odom_msg):
    # Create a TransformStamped message
    tf_msg = TransformStamped()
    tf_msg.header.stamp = odom_msg.header.stamp  # Timestamp from Odometry
    tf_msg.header.frame_id = odom_msg.header.frame_id  # Typically the "world" or "map" frame
    tf_msg.child_frame_id = odom_msg.child_frame_id if hasattr(odom_msg, 'child_frame_id') else "base_link"

    # Set translation
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
    tf_msg.transform.translation.z = odom_msg.pose.pose.position.z

    # Set rotation
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation

    # Wrap in TFMessage to publish
    tf_message = TFMessage(transforms=[tf_msg])
    tf_pub.publish(tf_message)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('odom_to_tf_publisher')

    # Publisher for tf messages
    tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

    # Subscriber to odometry messages
    rospy.Subscriber('/ground_truth/imu_tf', Odometry, odom_to_tf)

    # Spin to keep the node active
    rospy.spin()