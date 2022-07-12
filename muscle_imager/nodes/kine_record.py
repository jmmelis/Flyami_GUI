#!/usr/bin/env python

import rospy
import rosparam
import rospkg

import os
import time
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import Header, String, Bool, UInt32, Float32, UInt8

from Kinefly.msg import MsgFlystate

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import h5py

class KineflyRecord():

	def __init__(self):
		# Initialize the node
        rospy.init_node('kinefly_save_node')

        # CvBridge()
        self.cvbridge = CvBridge()

        # Buffers -------------------------------------------------------------------------
        self.buff_size = 1800 # record roughly 30 seconds of data
        self.kine_img_res =  [608, 808]

        # Parameters ----------------------------------------------------------------------

        self.img_frame_count 	= np.zeros(self.buff_size, dtype=np.uint32)
        self.img_buffer      	= np.zeros([self.ca_img_res[0],self.ca_img_res[1],self.buff_size])
        self.lmr_frame_count    = np.zeros(self.buff_size, dtype=np.uint32)
        self.l_wing_buffer      = np.zeros(self.buff_size, dtype=np.float64)
        self.r_wing_buffer      = np.zeros(self.buff_size, dtype=np.float64)
        self.lmr_wing_buffer    = np.zeros(self.buff_size, dtype=np.float64)

        # Subscribers ---------------------------------------------------------------------

        self.kine_image  = rospy.Subscriber('/kine_camera/image_mono',MsgLabeledImg,self.kine_image_callback)
        self.fly_flying  = rospy.Subscriber('/frame_counter/fly_flying',Bool,self.fly_flying_callback)
        self.frame_count = rospy.Subscriber('/frame_counter/frame_count',UInt32,self.frame_count_callback)
        self.phase 		 = rospy.Subscriber('/frame_counter/phase_msg',UInt32,self.wb_phase_callback)
        self.wb_period 	 = rospy.Subscriber('/frame_counter/wb_T_msg',UInt32,self.wb_period_callback)
        self.kine_data   = rospy.Subscriber('/kinefly/flystate',MsgFlystate,self.kine_data_callback)

        #-----------------------------------------------------------------------------------------------------------
        #
        # Now the actual program starts, when the experimental script publishes a message /exp_scripts/save_hdf5
        # all buffers and lists will be saved to an hdf5 file which name is specified by the save_hdf5 message.
        #
        #-----------------------------------------------------------------------------------------------------------

        self.save_hdf5 = rospy.Subscriber('/exp_scripts/save_hdf5',String,self.save_hdf5_callback)

    def save_hdf5_callback(self,msg):

        f = h5py.File(msg.data,'w')

        f.create_dataset('muscle/ref_frame',data=self.get_ref_frame)
        f.create_dataset('muscle/ca_image/frame_count',data=self.ca_img_frame_count)
        f.create_dataset('muscle/ca_image/image',data=self.ca_img_buffer)

        f.create_dataset('kinefly/frame_count',data=self.lmr_frame_count)
        f.create_dataset('kinefly/l_wing',data=self.l_wing_buffer)
        f.create_dataset('kinefly/r_wing',data=self.r_wing_buffer)
        f.create_dataset('kinefly/lmr_wing',data=self.lmr_wing_buffer)

        f.create_dataset('rig_state/fly_flying',data=self.fly_flying_buffer)
        f.create_dataset('rig_state/frame_count',data=self.frame_count_buffer)
        f.create_dataset('rig_state/pattern_frame',data=self.pattern_frame)
        f.create_dataset('rig_state/pattern_ind',data=self.pattern_buffer)
        f.create_dataset('rig_state/panel_mode_frame',data=self.panel_mode_frame)
        f.create_dataset('rig_state/panel_mode',data=self.panel_mode_buffer)

        f.close()

   	