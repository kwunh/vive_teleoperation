#!/usr/bin/env python

import rospy
import baxter_interface

import ik_solver
import robot
import subscriber
import robot_wrists

# Moves limbs to desired goal pose
def move_to_goal(limb):
    if (limb == 'left' and robot.pan_mode == False):
        robot.left_limb.set_joint_positions(ik_solver.ik_solve('left', robot.goalPose))
    elif (limb == 'right' and robot.pan_mode == False):
        robot.right_limb.set_joint_positions(ik_solver.ik_solve('right', robot.goalPose))

# Checks roll mode state for LH limb
def check_LH_roll():
    if robot.LH_roll_mode == False:
        robot_wrists.LH_rolling = False
        move_to_goal('left')
    else:
        robot_wrists.roll('left')
        robot_wrists.LH_rolling = True
    return


if __name__ == '__main__':

    # Initialise the ROS node
    rospy.init_node("Baxter_Subscriber")
    rospy.sleep(0.5)

    # Enable Baxter()
    rs = baxter_interface.RobotEnable()
    rs.enable()

    # Initialise robot to the given starting pose
    robot.initialise()

    # Initialise subscribers
    subscriber.listener()

    while not rospy.is_shutdown():

        # Loop until goal pose is achieved, according to precision tolerance
        # Checks roll mode state for RH limb
        while not robot.poseGoalReached():
            if robot.RH_roll_mode == False:
                robot_wrists.RH_rolling = False
                move_to_goal('right')
                check_LH_roll()
            else:
                robot_wrists.roll('right')
                robot_wrists.RH_rolling = True
                check_LH_roll()

    quit()
