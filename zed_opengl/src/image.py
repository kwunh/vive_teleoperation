#!/usr/bin/env python


import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from Queue import Queue


class ImageConverter(object):
	
	def __init__(self, object):

		self.topic=object
		self.bridge= CvBridge()
		self.image_sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=100)
		

	def callback(self, data):
		try:
			cv_image=self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		else:
            		get_image(self, cv_image)

def get_image(self, cv_image):
	
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)
        	
def subscribe(position):
	ic= ImageConverter(position)
	rospy.init_node('image_converter', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("goodbye")
	cv2.destroyAllWindows()


