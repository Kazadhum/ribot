#!/usr/bin/env python3

from turtle import position
from geometry_msgs.msg import Quaternion, Vector3
from rosgraph_msgs.msg import Clock
import rospy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header
from controller import Robot, Supervisor
import math
import random

def moveToPosition(position):
    for i in range(len(motors)):
        motors[i].setPosition(position[i])

robot = Supervisor()

timestep = 32
wait_time = 8  # secs
wait_steps = int((wait_time * 1000) / timestep)

joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

motors = []

for name in joint_names:
    motor = robot.getDevice(name)
    motor.setPosition(0.0)
    motor.setVelocity(0.1)
    motors.append(motor)

position_sequence = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]

# Get motor position sensors
sensors = []
for joint in joint_names:
    motor = robot.getDevice(joint)
    sensor = motor.getPositionSensor()
    sensor.enable(timestep)
    sensors.append(sensor)

# Enable sensors
accelerometer = robot.getDevice('accelerometer')
accelerometer.enable(timestep)
gyroscope = robot.getDevice('gyro')
gyroscope.enable(timestep)

random.seed(10)

for i in range(10):
    random_pos = [random.uniform(-2, 2) for k in range(len(motors))]
    position_sequence.append(random_pos)


position_idx = 0
wait_counter = 0
is_moving = True

moveToPosition(position_sequence[position_idx])

rospy.init_node(name="data_handler", anonymous=True)
joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

imu_pub = rospy.Publisher(name="imu", data_class=Imu, queue_size=10)

# Clock publisher
clock_pub = rospy.Publisher(name='clock', data_class=Clock, queue_size=1)

rate = rospy.Rate(1000/timestep)

while robot.step(timestep) != -1 and not rospy.is_shutdown():

    # Publish joint states
    joint_msg = JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = joint_names
    joint_msg.position = [sensor.getValue() for sensor in sensors]
    
    joint_pub.publish(joint_msg)

    # Read the sensors:    
    gyro_values = gyroscope.getValues()
    accel_values = accelerometer.getValues()
    
    imu_msg = Imu()
    
    # Header
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'accelerometer'
    # Orientation (ignore)
    imu_msg.orientation = Quaternion(0, 0, 0, 1)
    imu_msg.orientation_covariance = [0] * 9
    imu_msg.orientation_covariance[0] = -1 # Ignore orientation
    # Linear Acceleration
    imu_msg.linear_acceleration = Vector3(*accel_values)
    imu_msg.linear_acceleration_covariance = [0] * 9
    # Angular Velocity
    imu_msg.angular_velocity = Vector3(*gyro_values)
    imu_msg.angular_velocity_covariance = [0] * 9

    imu_pub.publish(imu_msg)

    # Publish /clock
    sim_time = robot.getTime()
    secs = int(sim_time)
    nsecs = int((sim_time - secs) * 1e9)

    clock_msg = Clock()
    clock_msg.clock.secs = secs
    clock_msg.clock.nsecs = nsecs
    clock_pub.publish(clock_msg)

    if is_moving:
        wait_counter += 1
        if wait_counter >= wait_steps:
            position_idx += 1
            if position_idx < len(position_sequence):
                moveToPosition(position_sequence[position_idx])
                wait_counter = 0
