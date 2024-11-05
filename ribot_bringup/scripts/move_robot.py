#!/usr/bin/env python3

import rospy
import tf
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import time

import tf.transformations

def update_gazebo_model(pose):
    """
    Update the model in Gazebo based on the provided pose.
    """
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()

        # Name of the model you want to update in Gazebo (adjust this to your model name)
        model_state.model_name = 'robot'

        # Apply the new pose from the TF to the Gazebo model
        model_state.pose = pose

        # Set velocity to zero (we are controlling the position manually)
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0

        # Keep the reference frame the same (world)
        model_state.reference_frame = 'world'

        # Call the service to update the model state in Gazebo
        resp = set_model_state(model_state)

        rospy.loginfo("Gazebo model updated to new position: {}".format(pose.position))

    except rospy.ServiceException as e:
        rospy.logerr("Failed to update model state in Gazebo: {}".format(e))


def tf_to_gazebo_transform():
    """
    Main loop to listen to TF transforms and update the Gazebo model.
    """
    rospy.init_node('tf_to_gazebo')

    # Create a TF listener
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10 Hz update rate

    # Define Poses
    # Orientations in Euler Angles: [r,p,y]
    target_poses = {
        "1": {
            "position": [-1, 0, 1],
            "orientation": [0, 0, 0]
        },
        "2": {
            "position": [-1, 1, 1],
            "orientation": [0, 0, -3.14/6]
        },
        "3": {
            "position": [-1, -1, 1],
            "orientation": [0, 0, 3.14/6]
        },
        "4": {
            "position": [-1, 0, 1],
            "orientation": [0, 0, 0]
        },
        "5": {
            "position": [-1, 0.5, 1],
            "orientation": [0, 0, -3.14/12]
        },
        "6": {
            "position": [-1, -0.5, 1],
            "orientation": [0, 0, 3.14/12]
        },
        "7": {
            "position": [-1, 0, 1],
            "orientation": [0, 0, 0]
        },
        "8": {
            "position": [-1, 0, 1],
            "orientation": [3.14/12, 0, 0]
        },
        "9": {
            "position": [-1, 0, 1],
            "orientation": [3.14/6, 0, 0]
        },
        "10": {
            "position": [-1, 0, 1],
            "orientation": [-3.14/12, 0, 0]
        },
        "11": {
            "position": [-1, 0, 1],
            "orientation": [-3.14/6, 0, 0]
        },
        "12": {
            "position": [-1, 1, 1],
            "orientation": [3.14/12, 0, -3.14/6]
        },
        "13": {
            "position": [-1, 0, 1],
            "orientation": [-3.14/12, 0, 0]
        },
        "14": {
            "position": [-1, 0, 1],
            "orientation": [-3.14/6, 0, 0]
        },
    }

    while not rospy.is_shutdown():
        
        for target_pose_key, target_pose in target_poses.items():

            # Convert euler angles to quats
            roll = target_pose["orientation"][0]
            pitch = target_pose["orientation"][1]
            yaw = target_pose["orientation"][2]
            
            quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            print(quat)

            # Create a Pose object from the TF data
            pose = Pose()
            pose.position.x = target_pose["position"][0]
            pose.position.y = target_pose["position"][1]
            pose.position.z = target_pose["position"][2]
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]


            # Update Gazebo with the new pose
            update_gazebo_model(pose)

            time.sleep(3)

            # Sleep for the loop rate
            rate.sleep()
        
        # Sleep for the loop rate
        rate.sleep()

        
if __name__ == '__main__':
    try:
        tf_to_gazebo_transform()
    except rospy.ROSInterruptException:
        pass
