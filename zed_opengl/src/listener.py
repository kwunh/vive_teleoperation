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
		self.image_queue = Queue(maxsize=100)
		self.image_sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=100)
		self.image=None

	def callback(self, data):
		try:
			cv_image=self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		else:
            		self.image = cv_image
            		self.image_queue.put(cv_image)

	def get_image(self):

        	try:
            		image = self.image_queue.get(block=False)
        	except:
            		image = self.image
		
		cv2.imshow("Image window", image)
		cv2.waitKey(3)
        	
def subscribe(position):
	ic= ImageConverter(position)
	ic.get_image()
	rospy.init_node('image_converter', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("goodbye")
	cv2.destroyAllWindows()


