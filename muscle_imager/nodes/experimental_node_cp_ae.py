#!/usr/bin/env python

import rospy
import rosparam
import rospkg

from std_msgs.msg import Header, String, Bool, UInt32
#import sys
#print sys.path

import numpy as np
import h5py
import os
import cv2
import random

from ledpanels import display_ctrl

class ExperimentalNode(object):

	def __init__(self):
		# Initialize exp_script node
		rospy.init_node('exp_script')

		#self.rate = rospy.Rate(50) #50 Hz

		# Experimental variables
		# self.pattern_names = ['loom left', 'loom right', 'loom back', 'loom front', 'pitch up', 'pitch down', 
		# 				'roll right', 'roll left', 'yaw left', 'yaw right', 'trans right', 'trans left',
		# 				'trans front', 'trans back', 'trans up', 'trans down']
		# self.pattern_indices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

		self.pattern_names = ['loom left', 'loom right', 'loom back', 'loom front', 'pitch up', 'pitch down', 
						'roll right', 'roll left', 'yaw left', 'yaw right', 'trans right', 'trans left',
						'trans front', 'trans back', 'trans up', 'trans down']
		self.pattern_indices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
		self.pattern_id = 1
		self.CL_duration = 60


		# self.closed_loop_gain = 2.0
		self.closed_loop_gain = 13
		self.closed_loop_bias = 0
		self.open_loop_gain = 0
		self.open_loop_bias = 0

		self.current_mode = 'off'

		# Load led controller
		rospy.logwarn('loading display control')
		self.ctrl = display_ctrl.LedControler()
		rospy.sleep(10.0)
		rospy.logwarn('all panels on')
		self.ctrl.all_on()
		rospy.sleep(5.0)
		rospy.logwarn('all panels off')
		self.ctrl.all_off()
		rospy.sleep(1.0)
		rospy.logwarn('setting channel 3')

		# Set the analog channel 3 to 0.0 V
		rospy.sleep(1.0)
		self.ctrl.set_ao(3,value = 0.0)
		rospy.logwarn('setting channel 4')

		# Set the analog channel 4 to 0.0 V
		rospy.sleep(1.0)
		self.ctrl.set_ao(4,value = 0.0)

		rospy.logwarn('finished')

		# Subscribe to fly_flying topic
		self.cmd_sub = rospy.Subscriber('/muscles/panel_cmd', String, self.LED_cmd_callback)

		# Experimental state publisher
		self.state_pub = rospy.Publisher('/exp_script/exp_state', String, queue_size = 10)

		# Pattern publisher
		self.pattern_pub = rospy.Publisher('/exp_script/pattern', String, queue_size = 10)

	def loom_only_mode(self):
		self.pattern_indices = [1,2,3,4]

	def trans_only_mode(self):
		self.pattern_indices = [11,12,13,14,15,16]

	def rot_only_mode(self):
		self.pattern_indices = [5,6,7,8,9,10]

	def trans_rot_mode(self):
		self.pattern_indices = [5,6,7,8,9,10,11,12,13,14,15,16]

	def loom_trans_rot_mode(self):
		self.pattern_indices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]

	def opto_mode(self):
		self.pattern_indices = [5,6,7,8,9,10,11,12,13,14,15,16]

	def ivo_mode(self):
		self.pattern_indices = [1]

	def select_random_pattern(self):
		select_ind = self.pattern_indices.index(random.choice(self.pattern_indices))
		self.pattern_id = self.pattern_indices[select_ind]
		rospy.logwarn('pattern_id: ' + str(self.pattern_id))

	def set_closed_loop_gain(self,gain_in):
		self.closed_loop_gain = gain_in
		#self.closed_loop_mode()

	def set_closed_loop_bias(self,bias_in):
		self.closed_loop_bias = bias_in

	def set_open_loop_gain(self,gain_in):
		self.open_loop_gain = gain_in
		#self.open_loop_mode()

	def set_open_loop_bias(self,bias_in):
		self.open_loop_bias = bias_in

	def set_pattern(self):
		self.ctrl.stop()
		rospy.sleep(0.001)
		self.ctrl.set_pattern_id(self.pattern_id)
		rospy.sleep(0.001)

	def closed_loop_mode(self):
		self.ctrl.stop()
		# # set random pattern
		# self.select_random_pattern()
		# self.set_pattern()
		# set pattern (test here)
		self.set_pattern()
		# set closed loop mode
		# self.ctrl.set_position(np.random.randint(0,192),0) #randomize x position, use 0 position y channel
		rospy.sleep(0.001)
		self.ctrl.set_mode('xrate=ch0','yrate=funcy')
		rospy.sleep(0.001)
		# moved position to after mode set
		self.ctrl.set_position(np.random.randint(0,192),0) #randomize x position, use 0 position y channel
		rospy.sleep(0.001)
		self.ctrl.send_gain_bias(gain_x=self.closed_loop_gain,bias_x=self.closed_loop_bias,gain_y=0,bias_y=0)
		rospy.sleep(0.001)
		self.ctrl.start()
		# added longer duration
		rospy.sleep(self.CL_duration)
		rospy.sleep(0.001)

	def open_loop_mode(self):
		# open loop mode
		self.ctrl.set_position(np.random.randint(0,192),1) #randomize x position, use 1 position y channel
		rospy.sleep(0.001)
		self.ctrl.set_mode('xrate=funcx','yrate=funcy')
		rospy.sleep(0.001)
		self.ctrl.send_gain_bias(gain_x=self.open_loop_gain,bias_x=self.open_loop_bias,gain_y=0,bias_y=0)
		rospy.sleep(0.001)
		self.ctrl.start()
		#rospy.sleep(self.open_loop_duration)
		#self.ctrl.stop()
		#rospy.sleep(0.01)

	def static_mode(self):
		self.ctrl.stop()
		# set random pattern
		self.select_random_pattern()
		self.set_pattern()
		# set closed loop mode
		self.ctrl.set_position(np.random.randint(0,192),0) #randomize x position, use 0 position y channel
		rospy.sleep(0.001)
		self.ctrl.set_mode('xrate=ch0','yrate=funcy')
		#rospy.sleep(0.001)
		#self.ctrl.send_gain_bias(gain_x=0,bias_x=0,gain_y=0,bias_y=0)
		#rospy.sleep(0.001)
		#self.ctrl.start()

	def fire_trigger_pulse(self):
		self.ctrl.set_ao(3,value = 5.0)
		rospy.sleep(0.0001)
		self.ctrl.set_ao(3,value = 0.0)
		rospy.sleep(0.0001)

	def start_experiment(self):
		self.ctrl.set_ao(4,value = 0.0)

	def stop_experiment(self):
		self.ctrl.set_ao(4,value = 0.0)

	def opto_stimulus(self):
		self.ctrl.set_ao(4,value=5.0)
		rospy.sleep(0.25)
		#rospy.sleep(0.125)
		self.ctrl.set_ao(3,value=5.0)
		rospy.sleep(0.0001)
		self.ctrl.set_ao(3,value=0.0)
		rospy.sleep(0.25)
		#rospy.sleep(0.0001)
		self.ctrl.set_ao(4,value=0.0)
		rospy.sleep(0.0001)

	def LED_cmd_callback(self,msg):
		if msg.data == 'loom_only_mode':
			self.loom_only_mode()
		elif msg.data == 'trans_only_mode':
			self.trans_only_mode()
		elif msg.data == 'rot_only_mode':
			self.rot_only_mode()
		elif msg.data == 'trans_rot_mode':
			self.trans_rot_mode()
		elif msg.data == 'loom_trans_rot_mode':
			self.loom_trans_rot_mode()
		elif msg.data == 'open_loop':
			if (self.current_mode is not 'open_loop'):
				self.open_loop_mode()
				self.current_mode = 'open_loop'
		elif msg.data == 'closed_loop':
			if (self.current_mode is not 'closed_loop'):
				self.closed_loop_mode()
				self.current_mode = 'closed_loop'
		elif msg.data == 'fire_trigger':
			self.fire_trigger_pulse()
		elif msg.data == 'off':
			if (self.current_mode is not 'off'):
				self.current_mode = 'off'
				self.ctrl.all_off()
				rospy.logwarn('all panels off')
		elif msg.data == 'start_exp':
			self.start_experiment()
		elif msg.data == 'stop_exp':
			self.stop_experiment()
		elif msg.data == 'opto_mode':
			self.opto_mode()
		elif msg.data == 'ivo_mode':
			self.ivo_mode()
			self.static_mode()
		elif msg.data == 'opto_pulse':
			self.opto_stimulus()
		elif 'closed_loop_gain:' in msg.data :
			split_txt = msg.data.split(':')
			gain_in = float(split_txt[1])
			self.set_closed_loop_gain(gain_in)
			rospy.logwarn('closed_loop_gain:' + str(gain_in))
		elif 'open_loop_gain:' in msg.data :
			split_txt = msg.data.split(':')
			gain_in = float(split_txt[1])
			self.set_open_loop_gain(gain_in)
			rospy.logwarn('open_loop_gain:' + str(gain_in))

if __name__ == '__main__':
	expnode = ExperimentalNode()
	# rospy.spin()
	expnode.closed_loop_mode()

