#!/usr/bin/env python

"""
ROS subscribers used to extract PoseStamped and Joy messages from controllers
ROS subscriber used to extract tf messages from HMD (for head panning)

"""

import rospy
import robot
import constants

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage

from tf.transformations import euler_from_quaternion

# Callback function to process translation and rotation data from left controller
def left_pose_cb(data):
    robot.goalPose.LH_pos_x = data.pose.position.x * constants.HUMAN_ROBOT_MAPPING * constants.FORWARD_AXIS_CORRECTION
    robot.goalPose.LH_pos_y = data.pose.position.y * constants.HUMAN_ROBOT_MAPPING
    robot.goalPose.LH_pos_z = (data.pose.position.z + constants.VIVE_ORIGIN_ADJUST) * constants.HUMAN_ROBOT_MAPPING

    quaternion_angles= (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

    # Convert to euler angles for easier co-ordinate conversions
    euler_angles= euler_from_quaternion(quaternion_angles)

    robot.goalPose.LH_roll = -euler_angles[2] 
    robot.goalPose.LH_pitch = -euler_angles[1] + constants.RIGHT_ANGLE_RAD
    robot.goalPose.LH_yaw = -euler_angles[0] 

    ##Rospy.loginfo(rospy.get_caller_id() + "Left controller pose data: %s", data.pose)


# Callback function to process translation and rotation data from left controller
def right_pose_cb(data):
    robot.goalPose.RH_pos_x = data.pose.position.x * constants.HUMAN_ROBOT_MAPPING * constants.FORWARD_AXIS_CORRECTION
    robot.goalPose.RH_pos_y = data.pose.position.y * constants.HUMAN_ROBOT_MAPPING
    robot.goalPose.RH_pos_z = (data.pose.position.z + constants.VIVE_ORIGIN_ADJUST) * constants.HUMAN_ROBOT_MAPPING

    quaternion_angles= (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)

    # Convert to euler angles for easier conversions
    euler_angles= euler_from_quaternion(quaternion_angles)

    robot.goalPose.RH_roll = -euler_angles[2] 
    robot.goalPose.RH_pitch = -euler_angles[1] + constants.RIGHT_ANGLE_RAD
    robot.goalPose.RH_yaw = -euler_angles[0]

    ##rospy.loginfo(rospy.get_caller_id() + "Right controller pose data: %s", data.pose)


# Callback function to process button & controller inputs from left controller
def left_joy_cb(data):

    # Left trigger controls left gripper
    trigger_axis = data.axes[0]
    robot.update_left_gripper(100.0 - (trigger_axis*100.0))

    # Roll mode for LH activated when left trackpad pressed
    # LH limb locked whilst roll mode is active
    roll_button=data.buttons[2]
    if (roll_button == 1.0):
        robot.LH_roll_mode = True
    else:
        robot.LH_roll_mode = False

    # Pan mode for robot's head activated when menu button (on left controller) pressed
    # Both limbs are locked whilst pan mode is active
    pan_button=data.buttons[3]
    if (pan_button == 1.0):
        robot.pan_mode = True
    else:
        robot.pan_mode = False


# Callback function to process button & controller inputs from right controller
def right_joy_cb(data):

    # Right trigger controls right gripper
    trigger_axis = data.axes[0]
    robot.update_right_gripper(100.0 - (trigger_axis*100.0))

    # Roll mode for RH activated when right trackpad pressed
    # RH limb locked whilst roll mode is active
    grip_button=data.buttons[2]
    if (grip_button == 1.0):
        robot.RH_roll_mode = True
    else:
        robot.RH_roll_mode = False

    reset_button=data.buttons[3]
    if (reset_button == 1.0):
        robot.reset_pose()


# Callback function to pan robot's head
def pan_head(data):
    while (robot.pan_mode== True):
        # head pan is rotation in the z axis
        pan = data.transforms[0].transform.rotation.z
        if (constants.HEAD_PAN_MIN<= pan <= constants.HEAD_PAN_MAX):
            robot.update_head_pan(pan)
            #robot.head.set_pan(pan, timeout=5.0)


# Subscribe to topics to receive data from HTC publishers
def listener():

    left_pose= rospy.Subscriber('/left_controller_as_posestamped', PoseStamped, left_pose_cb, queue_size=1)

    right_pose= rospy.Subscriber('/right_controller_as_posestamped', PoseStamped, right_pose_cb, queue_size=1)

    left_joy= rospy.Subscriber('/vive_left', Joy, left_joy_cb, queue_size=1)

    right_joy = rospy.Subscriber('/vive_right', Joy, right_joy_cb, queue_size=1)

    hmd_pose= rospy.Subscriber('/tf', TFMessage, pan_head, queue_size=1)
