#!/usr/bin/env python

"""
Predefined constants required for mapping human to robot
Values can be adjusted as necessary

"""
import cv2
import cv_bridge
import math
import rospy
from sensor_msgs.msg import Image

# Robot joint speed (0.0 - 1.0)
JOINT_SPEED = 1.0

# Velocity of the robot gripper (0.0 - 100.0)
GRIP_VELOCITY = 100

# Threshold of accuracy between target pose and actual pose
PRECISION_TOLERANCE = 0.05

# 90 degrees rotation for pitch rotation mapping
RIGHT_ANGLE_RAD = 1.5708

# For user-robot mapping (in m)
HUMAN_ARMS_OUTSTRETCHED = 0.8
ROBOT_ARMS_OUTSTRETCHED= 1.45
HUMAN_ROBOT_MAPPING= ROBOT_ARMS_OUTSTRETCHED/HUMAN_ARMS_OUTSTRETCHED

# Distance between robot's shoulder and base
ROBOT_SHOULDER_TO_BASE= 0.32

# Distance between user's shoulder and hmd (assume mounted on head)
HUMAN_SHOULDER_TO_HMD= abs(-0.30)

# Distance between user's shoulder and world
HUMAN_SHOULDER_TO_WORLD= abs(1.40)

# Adjust origin of vive's co-ordinate frame to robot's co-ordinate frame
def calculate_offset(origin):
    alpha=math.atan(ROBOT_SHOULDER_TO_BASE/ROBOT_ARMS_OUTSTRETCHED)
    user_shoulder_to_base= math.tan(alpha) * HUMAN_ARMS_OUTSTRETCHED
    if (origin == "hmd"):
        offset= user_shoulder_to_base + HUMAN_SHOULDER_TO_HMD
    elif (origin == "world"):
        offset = user_shoulder_to_base + HUMAN_SHOULDER_TO_WORLD
    return offset

VIVE_ORIGIN_ADJUST= calculate_offset("hmd")

# Arbitrary value to make it less exhausting for the user to operate with arms forwards
FORWARD_AXIS_CORRECTION= 1.1

# Minimum and maximum rotation values of the robot in radians
WRIST_ROLL_MIN = -3.059
WRIST_ROLL_MAX = 3.059

HEAD_PAN_MIN= -1.3
HEAD_PAN_MAX= 1.3


