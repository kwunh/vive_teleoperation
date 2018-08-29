#!/usr/bin/env python

from tf.transformations import quaternion_from_euler

if __name__ == '__main__':

	q = quaternion_from_euler(0, 1.5707, 0)
	print "quat is %s %s %s %s." % (q[0], q[1], q[2], q[3])

