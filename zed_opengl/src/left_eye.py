#!/usr/bin/env python

import thread
import image


ROS_TOPIC_LEFT = "/zed/left/image_rect_color"

image.subscribe(ROS_TOPIC_LEFT)
