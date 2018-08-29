#!/usr/bin/env python

# ROS
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage


def left_cb(data):
    rospy.loginfo(rospy.get_caller_id() +"Left Controller- Position X %s", data.pose.position.x)
    #rospy.loginfo("Left Controller- Position Y %s", data.pose.position.y)
    #rospy.loginfo("Left Controller- Position Z %s", data.pose.position.z)

    #rospy.loginfo("Left Controller- Rotation X %s", data.pose.orientation.x)
    #rospy.loginfo("Left Controller- Rotation Y %s", data.pose.orientation.y)
    #rospy.loginfo("Left Controller- Rotation Z %s", data.pose.orientation.z)

def right_cb(data):
    rospy.loginfo(rospy.get_caller_id() +"Right Controller- Position X %s", data.pose.position.x)
    #rospy.loginfo("Left Controller- Position Y %s", data.pose.position.y)
    #rospy.loginfo("Left Controller- Position Z %s", data.pose.position.z)

    #rospy.loginfo("Left Controller- Rotation X %s", data.pose.orientation.x)
    #rospy.loginfo("Left Controller- Rotation Y %s", data.pose.orientation.y)
    #rospy.loginfo("Left Controller- Rotation Z %s", data.pose.orientation.z)

def right_grip(data):
    rospy.loginfo(rospy.get_caller_id() + "RH Grip: %s", data.axes[0])
    trigger_axis= data.axes[0]

def left_grip(data):
    rospy.loginfo(rospy.get_caller_id() + "LH Grip: %s", data.axes[0])
    trigger_axis = data.axes[0]

def hmd_cb(data):
    rospy.loginfo("HMD Translation X: %s", data.transforms[0].transform.translation.x)
    #rospy.loginfo("HMD Translation Y: %s", data.transforms[0].transform.translation.y)
    #rospy.loginfo("HMD Translation Z: %s", data.transforms[0].transform.translation.z)

    #rospy.loginfo("HMD Rotation X: %s", data.transforms[0].transform.rotation.x)
    #rospy.loginfo("HMD Rotation Y: %s", data.transforms[0].transform.rotation.y)
    #rospy.loginfo("HMD Rotation Z: %s", data.transforms[0].transform.rotation.z)

def pan_head(data):
    rospy.loginfo(rospy.get_caller_id() + "HMD POS: %s", data.transform)

    
def listener():

    rospy.init_node('listener', anonymous=True)

    left_pose= rospy.Subscriber('/left_controller_as_posestamped', PoseStamped, left_cb, queue_size=1)

    #right_pose= rospy.Subscriber('/right_controller_as_posestamped', PoseStamped, right_cb, queue_size=1)

    #left_gripper = rospy.Subscriber('/vive_left', Joy, left_grip)

    #right_gripper = rospy.Subscriber('/vive_right', Joy, right_grip)

    #hmd_pose = rospy.Subscriber('/vive_hmd', TFMessage, pan_head, queue_size=1)

    #hmd= rospy.Subscriber('/tf', TFMessage, hmd_cb, queue_size=1)

    rospy.spin() 

if __name__ == '__main__':
    listener()
