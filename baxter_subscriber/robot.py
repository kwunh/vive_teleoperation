#!/usr/bin/env python

"""
Robot's pose and limb interface initialisation

"""

import baxter_interface
import constants

# Holds values for position and orientation of robot's end-effectors
# Required for setting current poses and goal poses
class RobotPose:
    def __init__(self):
        # Initial values for pose replicating human stance
        self.RH_pos_x = 0.6342171236731038
        self.RH_pos_y = -0.36043150138041063
        self.RH_pos_z = 0.09859203828689367

        self.RH_ori_w = 0.7455594233663007
        self.RH_ori_x = -0.12293476834953236
        self.RH_ori_y = 0.6495988568385836
        self.RH_ori_z = 0.08396138490140141

        self.RH_roll = -30.939090668751863
        self.RH_pitch = 81.58539975618045
        self.RH_yaw = -14.12142637160184

        self.LH_pos_x = 0.6329613779972973
        self.LH_pos_y = 0.38766854601727874
        self.LH_pos_z = 0.09283436065295993

        self.LH_ori_w = 0.6965639109812897
        self.LH_ori_x = 0.1279323909724876
        self.LH_ori_y = 0.7019381984496922
        self.LH_ori_z = -0.07559620900358009

        self.LH_roll = -30.939090668751863
        self.LH_pitch = 81.58539975618045
        self.LH_yaw = -14.12142637160184


def initialise():
    global right_limb
    global left_limb
    global right_gripper
    global left_gripper
    global RH_roll_mode
    global LH_roll_mode
    global pan_mode
    global head
    global goalPose
    global currentPose


    # Initialise the robot's limbs and grippers
    right_limb = baxter_interface.Limb("right")
    left_limb = baxter_interface.Limb("left")
    right_gripper = baxter_interface.Gripper('right')
    left_gripper = baxter_interface.Gripper('left')

    # Set limb and gripper parameters
    right_limb.set_joint_position_speed(constants.JOINT_SPEED)
    left_limb.set_joint_position_speed(constants.JOINT_SPEED)

    right_gripper.set_velocity(constants.GRIP_VELOCITY)
    right_gripper.calibrate()

    left_gripper.set_velocity(constants.GRIP_VELOCITY)
    left_gripper.calibrate()

    # Wrist roll mode set to False initially
    RH_roll_mode = False
    LH_roll_mode = False

    # Set up head
    head = baxter_interface.Head()
    head.set_pan(0)
    head.command_nod()

    # Head pan mode set to False initially
    pan_mode = False

    # Initialise current and goal pose of the robot
    goalPose = RobotPose()
    currentPose = RobotPose()
    reset_pose()
    updateCurrentPose()

# Updates baxter's current pose
# Of limb's end effectors (grippers) with respect to base
def updateCurrentPose():
    currentPose.RH_pos_x = right_limb.endpoint_pose()["position"].x
    currentPose.RH_pos_y = right_limb.endpoint_pose()["position"].y
    currentPose.RH_pos_z = right_limb.endpoint_pose()["position"].z
    currentPose.RH_ori_w = right_limb.endpoint_pose()["orientation"].w
    currentPose.RH_ori_x = right_limb.endpoint_pose()["orientation"].x
    currentPose.RH_ori_y = right_limb.endpoint_pose()["orientation"].y
    currentPose.RH_ori_z = right_limb.endpoint_pose()["orientation"].z

    currentPose.LH_pos_x = left_limb.endpoint_pose()["position"].x
    currentPose.LH_pos_y = left_limb.endpoint_pose()["position"].y
    currentPose.LH_pos_z = left_limb.endpoint_pose()["position"].z
    currentPose.LH_ori_w = left_limb.endpoint_pose()["orientation"].w
    currentPose.LH_ori_x = left_limb.endpoint_pose()["orientation"].x
    currentPose.LH_ori_y = left_limb.endpoint_pose()["orientation"].y
    currentPose.LH_ori_z = left_limb.endpoint_pose()["orientation"].z

# Reset Baxter to a neutral position
# For replicating human movements
def reset_pose():
    rangles = {'right_s0': -0.30181072001645515, 'right_s1': -0.17180584824316633, 'right_w0': -1.3230584295511694,
               'right_w1': -1.5715633171886063, 'right_w2': -0.5250049246537829, 'right_e0': 0.24313595487983808,
               'right_e1': 2.16483038690329}
    langles = {'left_w0': 1.2655341500054664, 'left_w1': -1.4442429117941171, 'left_w2': 0.4555922940019679,
               'left_e0': -0.22971362298584072, 'left_e1': 2.2595537005552147, 'left_s0': 0.33555829734993425,
               'left_s1': -0.33670878294084833}


    right_limb.set_joint_position_speed(1)
    right_limb.move_to_joint_positions(rangles)
    right_limb.set_joint_position_speed(constants.JOINT_SPEED)
    print ("Setting right arm to neutral position")

    left_limb.set_joint_position_speed(1)
    left_limb.move_to_joint_positions(langles)
    left_limb.set_joint_position_speed(constants.JOINT_SPEED)
    print("Setting left arm to neutral position")

    updateCurrentPose()

# Controls gripper force of right end effector
def update_right_gripper(gripper_force):
    right_gripper.command_position(gripper_force)

# Controls gripper force of left end effector
def update_left_gripper(gripper_force):
    left_gripper.command_position(gripper_force)

def update_head_pan(pan):
    head.set_pan(pan, timeout=5.0)

# Checks for accuracy of desired goal pose vs. actual goal pose
def poseGoalReached():

    updateCurrentPose()
    RH_pos_x_GoalReached = (abs(currentPose.RH_pos_x - goalPose.RH_pos_x) <= constants.PRECISION_TOLERANCE)
    RH_pos_y_GoalReached = (abs(currentPose.RH_pos_y - goalPose.RH_pos_y) <= constants.PRECISION_TOLERANCE)
    RH_pos_z_GoalReached = (abs(currentPose.RH_pos_z - goalPose.RH_pos_z) <= constants.PRECISION_TOLERANCE)

    RH_ori_w_GoalReached = (abs(currentPose.RH_ori_w - goalPose.RH_ori_w) <= constants.PRECISION_TOLERANCE)
    RH_ori_x_GoalReached = (abs(currentPose.RH_ori_x - goalPose.RH_ori_x) <= constants.PRECISION_TOLERANCE)
    RH_ori_y_GoalReached = (abs(currentPose.RH_ori_y - goalPose.RH_ori_y) <= constants.PRECISION_TOLERANCE)
    RH_ori_z_GoalReached = (abs(currentPose.RH_ori_z - goalPose.RH_ori_z) <= constants.PRECISION_TOLERANCE)

    LH_pos_x_GoalReached = (abs(currentPose.LH_pos_x - goalPose.LH_pos_x) <= constants.PRECISION_TOLERANCE)
    LH_pos_y_GoalReached = (abs(currentPose.LH_pos_y - goalPose.LH_pos_y) <= constants.PRECISION_TOLERANCE)
    LH_pos_z_GoalReached = (abs(currentPose.LH_pos_z - goalPose.LH_pos_z) <= constants.PRECISION_TOLERANCE)

    LH_ori_w_GoalReached = (abs(currentPose.LH_ori_w - goalPose.LH_ori_w) <= constants.PRECISION_TOLERANCE)
    LH_ori_x_GoalReached = (abs(currentPose.LH_ori_x - goalPose.LH_ori_x) <= constants.PRECISION_TOLERANCE)
    LH_ori_y_GoalReached = (abs(currentPose.LH_ori_y - goalPose.LH_ori_y) <= constants.PRECISION_TOLERANCE)
    LH_ori_z_GoalReached = (abs(currentPose.LH_ori_z - goalPose.LH_ori_z) <= constants.PRECISION_TOLERANCE)

    if (RH_pos_x_GoalReached & RH_pos_y_GoalReached & RH_pos_z_GoalReached & RH_ori_w_GoalReached &
            RH_ori_x_GoalReached & RH_ori_y_GoalReached & RH_ori_z_GoalReached & LH_pos_x_GoalReached &
            LH_pos_y_GoalReached & LH_pos_z_GoalReached & LH_ori_w_GoalReached & LH_ori_x_GoalReached &
            LH_ori_y_GoalReached & LH_ori_z_GoalReached):
        return True
    else:
		return False
