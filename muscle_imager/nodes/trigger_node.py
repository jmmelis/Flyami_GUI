#!/usr/bin/env python

import rospy
import rosparam
import rospkg

from std_msgs.msg import Header, String, Bool, UInt16, UInt32
from sensor_msgs.msg import Image
from muscle_imager.msg import MsgTrigger

import numpy as np

class trigger_node(object):

	def __init__(self):
		# initialize the node
		#rospy.init_node('trigger_node')
		#self.namespace = rospy.get_namespace()
		#self.nodename = rospy.get_name().rstrip('/')

		self.pubTrigger = rospy.Publisher('trigger_node', UInt16, queue_size=100)

	def publish_hs_trigger(self):
		self.pubTrigger.publish(np.uint16(1))
		rospy.logwarn('trigger msg: ' + str(1))

	def publish_opto_trigger_high(self):
		self.pubTrigger.publish(np.uint16(2))
		rospy.logwarn('trigger msg: ' + str(2))

	def publish_opto_trigger_low(self):
		self.pubTrigger.publish(np.uint16(3))
		rospy.logwarn('trigger msg: ' + str(3))

if __name__ == '__main__':
	trgnd = trigger_node()
	rospy.spin()