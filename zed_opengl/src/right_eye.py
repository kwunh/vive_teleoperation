#!/usr/bin/env python

import thread
import image
import rospy
import cv2
import listener

ROS_TOPIC_RIGHT = "/zed/right/image_rect_color"

listener.subscribe(ROS_TOPIC_RIGHT)




