#!/usr/bin/env python

import rospy
import rosparam
import rospkg

import os
import time
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import Header, String, Bool, UInt32, Float32, UInt8

from muscle_imager.srv import SrvRefFrame
from muscle_imager.srv import SrvRefFrameRequest

from muscle_imager.msg import MsgExtractedSignal
from muscle_imager.msg import MsgLabeledImg
from muscle_imager.msg import MsgTriggerActivation
from muscle_imager.msg import MsgTrigger

from Kinefly.msg import MsgFlystate

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import h5py

class cam_record(object):

    def __init__(self):
        # Initialize the node
        rospy.init_node('cam_save_node')

        # CvBridge()
        self.cvbridge = CvBridge()

        # Save the configuration of the muscle model
        rospy.wait_for_service('/unmixer/RefFrameServer')
        self.get_ref_frame = rospy.ServiceProxy('/unmixer/RefFrameServer', SrvRefFrame)
        #rospy.wait_for_service('/muscles/RefFrameServer')
        #self.get_ref_frame = rospy.ServiceProxy('/muscles/RefFrameServer', SrvRefFrame)

        # Lists ---------------------------------------------------------------------------
        self.trigger_header = []
        self.trigger_frame_count = []

        # Buffers -------------------------------------------------------------------------

        self.buff_size = 1800 # record roughly 30 seconds of data
        self.ca_img_res =  [304, 404]

        # Muscle data
        self.muscle_names = ['b1','b2','b3','i1','i2','iii1','iii3','iii24','hg1','hg2','hg3','hg4','nm','pr','bkg']
        self.muscle_frame_count = dict()
        self.muscle_buffers = dict()
        self.muscle_mean = dict()
        self.muscle_std = dict()
        self.muscle_score = dict()

        for muscle in self.muscle_names:
            self.muscle_frame_count[muscle] = np.zeros(self.buff_size, dtype=np.uint32)
            self.muscle_buffers[muscle] = np.random.randn(self.buff_size)
            self.muscle_mean[muscle] = np.zeros(self.buff_size, dtype=np.float64)
            self.muscle_std[muscle] = np.ones(self.buff_size, dtype=np.float64)
            self.muscle_score[muscle] = np.zeros(self.buff_size, dtype=np.float64)

        # Ca image
        self.ca_img_frame_count = np.zeros(self.buff_size, dtype=np.uint32)
        self.ca_img_buffer      = np.zeros([self.ca_img_res[0],self.ca_img_res[1],self.buff_size])

        # Rig state
        self.fly_flying_buffer  = np.zeros(self.buff_size, dtype=np.uint8)
        self.frame_count_buffer = np.zeros(self.buff_size, dtype=np.uint32)
        self.pattern_frame      = np.zeros(self.buff_size, dtype=np.uint32)
        self.pattern_buffer     = np.zeros(self.buff_size, dtype=np.uint8)
        self.panel_mode_frame   = np.zeros(self.buff_size, dtype=np.uint32)
        self.panel_mode_buffer  = np.zeros(self.buff_size, dtype=np.uint8)

        # Kinefly:
        self.lmr_frame_count    = np.zeros(self.buff_size, dtype=np.uint32)
        self.l_wing_buffer      = np.zeros(self.buff_size, dtype=np.float64)
        self.r_wing_buffer      = np.zeros(self.buff_size, dtype=np.float64)
        self.lmr_wing_buffer    = np.zeros(self.buff_size, dtype=np.float64)

        # Trigger msg:
        self.trig_frame_count   = np.zeros(self.buff_size, dtype=np.uint32)
        self.trig_score         = np.zeros(self.buff_size, dtype=np.float64)
        self.trig_index         = np.zeros(self.buff_size, dtype=np.uint8)

        # Subscribers -------------------------------------------------------------------

        # Muscle subscribers
        self.muscle_subcribers = dict()

        for muscle in self.muscle_names:
            self.muscle_subcribers[muscle] = rospy.Subscriber('/unmixer/%s'%(muscle),
                                                            MsgExtractedSignal,
                                                            self.muscle_signal_callback,   
                                                            queue_size=None, 
                                                            buff_size=10, 
                                                            tcp_nodelay=True)

        # Ca image subscriber
        self.ca_image    = rospy.Subscriber('/unmixer/image_output',MsgLabeledImg,self.ca_image_callback)

        # Rig state
        self.fly_flying  = rospy.Subscriber('/frame_counter/fly_flying',Bool,self.fly_flying_callback)
        self.frame_count = rospy.Subscriber('/frame_counter/frame_count',UInt32,self.frame_count_callback)
        self.pattern     = rospy.Subscriber('/exp_scripts/pattern',UInt8,self.pattern_callback)
        self.panel_mode  = rospy.Subscriber('/exp_scripts/panel_mode',UInt8,self.panel_mode_callback)

        # Kinefly
        self.kine_data   = rospy.Subscriber('/kinefly/flystate',MsgFlystate,self.kine_data_callback)

        # Trigger message from muscle_trigger.py
        self.trig_msg    = rospy.Subscriber('/muscle_trigger/trig_msg',MsgTriggerActivation,self.trigger_msg_callback)

        # Trigger message from Teensy
        self.trigger_msg = rospy.Subscriber('/frame_counter/trigger_msg',MsgTrigger,self.trigger_callback)

        #-----------------------------------------------------------------------------------------------------------
        #
        # Now the actual program starts, when the experimental script publishes a message /exp_scripts/save_hdf5
        # all buffers and lists will be saved to an hdf5 file which name is specified by the save_hdf5 message.
        #
        #-----------------------------------------------------------------------------------------------------------

        self.save_hdf5 = rospy.Subscriber('/exp_scripts/save_hdf5',String,self.save_hdf5_callback)

    def save_hdf5_callback(self,msg):

        f = h5py.File(msg.data,'w')

        f.create_dataset('trigger/header',data=self.trigger_header)
        f.create_dataset('trigger/count',data=self.trigger_frame_count)

        f.create_dataset('trig_msg/frame_count',data=self.trig_frame_count)
        f.create_dataset('trig_msg/score',data=self.trig_score)
        f.create_dataset('trig_msg/index',data=self.trig_index)

        for muscle in self.muscle_names:
            f.create_dataset('muscle/%s/frame_count'%(muscle),data=self.muscle_frame_count[muscle])
            f.create_dataset('muscle/%s/value'%(muscle),data=self.muscle_buffers[muscle])
            f.create_dataset('muscle/%s/mean'%(muscle),data=self.muscle_mean[muscle])
            f.create_dataset('muscle/%s/std'%(muscle),data=self.muscle_std[muscle])
            f.create_dataset('muscle/%s/score'%(muscle),data=self.muscle_score[muscle])
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


    def trigger_callback(self,msg):
        self.trigger_header.append(msg.header)
        self.trigger_frame_count.append(msg.frame_count)

    def muscle_signal_callback(self,msg):
        # Add the muscle message to the buffers
        self.muscle_buffers[msg.muscle] = np.roll(self.muscle_buffers[msg.muscle],-1)
        self.muscle_buffers[msg.muscle][-1] = msg.value 
        self.muscle_mean[msg.muscle] = np.roll(self.muscle_mean[msg.muscle],-1)
        self.muscle_mean[msg.muscle][-1] = np.mean(self.muscle_buffers[msg.muscle])
        self.muscle_std[msg.muscle] = np.roll(self.muscle_std[msg.muscle],-1)
        self.muscle_std[msg.muscle][-1] = np.std(self.muscle_buffers[msg.muscle])
        self.muscle_score[msg.muscle] = np.roll(self.muscle_score[msg.muscle],-1)
        self.muscle_score[msg.muscle][-1] = (msg.value*1.0-self.muscle_mean[msg.muscle][-1]*1.0)/(self.muscle_std[msg.muscle][-1]*1.0)
        self.muscle_frame_count[msg.muscle] = np.roll(self.muscle_frame_count[msg.muscle], -1)
        self.muscle_frame_count[msg.muscle][-1] = msg.frame_count

    def pattern_callback(self,msg):
        # Record the pattern name and the frame count
        self.pattern_frame = np.roll(self.pattern_frame,-1)
        self.pattern_frame[-1] = self.frame_count_buffer[-1]
        self.pattern_buffer = np.roll(self.pattern_buffer,-1)
        self.pattern_buffer[-1] = msg.data

    def panel_mode_callback(self,msg):
        # Record the panel mode and the frame count
        self.panel_mode_frame = np.roll(self.panel_mode_frame,-1)
        self.panel_mode_frame[-1] = self.frame_count_buffer[-1]
        self.panel_mode_buffer = np.roll(self.panel_mode_buffer,-1)
        self.panel_mode_buffer[-1] = msg.data

    def fly_flying_callback(self,msg):
        # Record a 0 or a 1 whether the fly is not flying or flying
        self.fly_flying_buffer = np.roll(self.fly_flying_buffer,-1)
        if msg.data == True:
            self.fly_flying_buffer[-1] = 1
        else:
            self.fly_flying_buffer[-1] = 0
        self.fly_flying = msg.data

    def frame_count_callback(self,msg):
        # Record the frame count number
        self.frame_count_buffer = np.roll(self.frame_count_buffer,-1)
        self.frame_count_buffer[-1] = msg.data

    def trigger_msg_callback(self,msg):
        # Record the trigger score and trigger index
        self.trig_frame_count = np.roll(self.trig_frame_count,-1)
        self.trig_frame_count[-1] = msg.frame_count
        self.trig_score = np.roll(self.trig_score,-1)
        self.trig_score[-1] = msg.score
        self.trig_index = np.roll(self.trig_index,-1)
        self.trig_index[-1] = msg.score_ind

    def kine_data_callback(self,msg):
        # Extract the left, right and lmr stroke angles
        self.lmr_frame_count = np.roll(self.lmr_frame_count,-1)
        self.lmr_frame_count[-1] = self.frame_count_buffer[-1]
        self.l_wing_buffer = np.roll(self.l_wing_buffer,-1)
        self.r_wing_buffer = np.roll(self.r_wing_buffer,-1)
        self.lmr_wing_buffer = np.roll(self.lmr_wing_buffer,-1)
        if ((len(msg.left.angles) >0) and (len(msg.right.angles) >0)):
            self.l_wing_buffer[-1] = msg.left.angles[0]
            self.r_wing_buffer[-1] = msg.right.angles[0]
            self.lmr_wing_buffer[-1] = msg.left.angles[0]-msg.right.angles[0]
        else:
            self.l_wing_buffer[-1] = np.nan
            self.r_wing_buffer[-1] = np.nan
            self.lmr_wing_buffer[-1] = np.nan

    def ca_image_callback(self,msg):
        # Add the calcium image to the buffer and record the frame_count nr
        self.ca_img_frame_count = np.roll(self.ca_img_frame_count,-1)
        self.ca_img_frame_count[-1] = msg.frame_count
        self.ca_img_buffer      = np.roll(self.ca_img_buffer,-1,axis=2)
        self.ca_img_buffer[:,:,-1] = np.asarray(self.cvbridge.imgmsg_to_cv2(msg.image, 'passthrough').astype(float))

if __name__ == '__main__':
    cmrcrd = cam_record()
    rospy.spin()
