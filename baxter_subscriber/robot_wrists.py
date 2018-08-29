#!/usr/bin/env python

"""
Isolated roll mechanism for moving robot's wrists

"""

import robot
import copy
import constants

RH_rolling = False
LH_rolling = False

def roll (roll_limb):

    global LH_initial_angle
    global RH_initial_angle

    if roll_limb == 'left':
        limb = robot.left_limb
        mode = robot.LH_roll_mode
        rolling =LH_rolling
        wrist = 'left_w2'
    else:
        limb = robot.right_limb
        mode = robot.RH_roll_mode
        rolling = RH_rolling
        wrist = 'right_w2'

    currAngles = limb.joint_angles()
    currAnglesCopy = copy.deepcopy(currAngles)

    if roll_limb == 'left':
        if rolling == False:
            LH_initial_angle= limb.joint_angles()
        currAnglesCopy = add_roll(currAnglesCopy, LH_initial_angle, 'left')
    else:
        if rolling == False:
            RH_initial_angle = limb.joint_angles()
        currAnglesCopy = add_roll(currAnglesCopy, RH_initial_angle, 'right')

    while ((abs(currAngles[wrist]- currAnglesCopy[wrist]) >= constants.PRECISION_TOLERANCE/2) & mode == True):
        limb.set_joint_positions(currAnglesCopy)
        currAngles= limb.joint_angles()

    return

def add_roll(curr_joint_angles, initial_angle, limb):
    if limb == 'left':
        wrist = 'left_w2'
        yaw = robot.goalPose.LH_yaw
    else:
        wrist = 'right_w2'
        yaw = robot.goalPose.RH_yaw

    angles = curr_joint_angles

    if yaw + initial_angle[wrist] <= constants.WRIST_ROLL_MIN:
        angles[wrist] = constants.WRIST_ROLL_MIN
    elif yaw + initial_angle[wrist] >= constants.WRIST_ROLL_MAX:
        angles[wrist] = constants.WRIST_ROLL_MAX
    else:
        angles[wrist] = yaw + initial_angle[wrist]
    return angles
