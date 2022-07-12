#!/usr/bin/env python

import rospy
import rosparam
import rospkg

import time

from std_msgs.msg import Header, String, Bool, UInt32, Float32

from muscle_imager.msg import MsgExtractedSignal
from muscle_imager.msg import MsgLabeledImg
from muscle_imager.msg import MsgTriggerActivation

import numpy as np
import numpy.matlib
import h5py
import os

class muscle_trigger(object):

    def __init__(self):
        # Calculate mean and std muscle activity and decide whether to fire a trigger
        rospy.init_node('muscle_trigger')
        self.namespace = rospy.get_namespace()
        self.nodename = rospy.get_name().rstrip('/')

        # Subscribe to fly_flying topic
        self.fly_flying = rospy.Subscriber('/frame_counter/fly_flying', Bool, self.fly_flying_callback)

        # Subscribe to downloading topic
        self.downloading = rospy.Subscriber('/frame_counter/downloading', Bool, self.downloading_callback)

        # Subscribe to frame_count topic
        self.frame_count = rospy.Subscriber('/frame_counter/frame_count', UInt32, self.frame_count_callback)

        # Subscribe to muscle topics
        self.muscle_names = ['b1','b2','b3','i1','i2','iii1','iii3','iii24','hg1','hg2','hg3','hg4']
        self.muscle_buffers = dict()
        self.muscle_mean = dict()
        self.muscle_std = dict()
        self.muscle_score = dict()
        self.muscle_subcribers = dict()
        self.muscle_buffer_samples = 1000
        self.min_std = 0.01

        for muscle in self.muscle_names:
            self.muscle_buffers[muscle] = np.random.randn(self.muscle_buffer_samples)
            self.muscle_mean[muscle] = 0.0
            self.muscle_std[muscle] = 1000.0
            self.muscle_score[muscle] = 0.0
            self.muscle_subcribers[muscle] = rospy.Subscriber('/unmixer/%s'%(muscle),
                                                            MsgExtractedSignal,
                                                            self.muscle_signal_callback,   
                                                            queue_size=None, 
                                                            buff_size=1000, 
                                                            tcp_nodelay=True)

        # Load activation matrix
        activation_file_name = rospy.get_param('/muscle_trigger/trigger_file','i am not set!!!!!')
        rospy.logwarn('muscle trigger file: {}'.format(activation_file_name))
        #print activation_file_name
        activation_data = h5py.File(activation_file_name,'r')
        activation_dict = dict()
        for key in activation_data.keys():
            activation_dict[key] = np.array(activation_data.get(key))
        self.activation_thresh = activation_dict['act_thresh']
        self.activation_matrix = np.empty([self.activation_thresh.shape[0],len(self.muscle_names)])
        for muscle in self.muscle_names:
            ind = self.muscle_names.index(muscle)
            self.activation_matrix[:,ind] = activation_dict[muscle]
        activation_data.close()

        # Trigger signal publisher
        #rospy.logwarn('creating muscle publisher')
        self.trig_pub = rospy.Publisher(self.nodename+'/trig_msg', 
                                        MsgTriggerActivation, queue_size=1)
        time.sleep(10.0)
        rospy.logwarn('starting muscle trigger')
        # Start the muscle evaluation
        self.muscle_img_subscriber = rospy.Subscriber('/unmixer/image_output',
                                                    MsgLabeledImg,
                                                    self.trigger_callback)


    def muscle_signal_callback(self,msg):
        self.muscle_buffers[msg.muscle] = np.roll(self.muscle_buffers[msg.muscle],-1)
        self.muscle_buffers[msg.muscle][-1] = msg.value 
        self.muscle_mean[msg.muscle] = np.mean(self.muscle_buffers[msg.muscle])
        m_std = np.std(self.muscle_buffers[msg.muscle])
        if m_std > self.min_std:
            self.muscle_std[msg.muscle] = m_std
        else:
            self.muscle_std[msg.muscle] = self.min_std
        self.muscle_score[msg.muscle] = (msg.value*1.0-self.muscle_mean[msg.muscle]*1.0) #/(self.muscle_std[msg.muscle]*1.0)

    def frame_count_callback(self,msg):
        self.frame_count = msg.data

    def fly_flying_callback(self,msg):
        self.fly_flying = msg.data

    def downloading_callback(self,msg):
        self.downloading = msg.data

    def trigger_callback(self,img):
        if self.fly_flying and not self.downloading:
            if self.muscle_mean:
                #rospy.logwarn('trig call')
                #t_start = time.time()
                header = Header(stamp=rospy.Time.now())
                act_mat = self.activation_matrix
                act_tresh = self.activation_thresh
                muscle_std_vec = np.vstack([self.muscle_std[muscle] for muscle in self.muscle_names])
                muscle_score_vec = np.vstack([self.muscle_score[muscle] for muscle in self.muscle_names])
                trig_score = np.dot(act_mat,muscle_score_vec)-np.multiply(act_tresh,np.matlib.repmat(muscle_std_vec,2,1))
                trig_max = np.amax(trig_score)
                trig_max_ind = np.argmax(trig_score)
                self.trig_pub.publish(header=header,
                                    frame_count=np.uint32(self.frame_count),
                                    score_ind=np.uint16(trig_max_ind),
                                    score=np.float32(trig_max))
                #rospy.logwarn('trig calc time: ' + str(time.time()-t_start))

'''    def trigger_callback(self,img):
        if self.fly_flying == True and self.downloading == False:
            header = Header(stamp=rospy.Time.now())
            act_mat = self.activation_matrix
            act_tresh = self.activation_thresh
            muscle_score_vec = np.vstack([self.muscle_score[muscle] for muscle in self.muscle_names])
            trig_score = np.dot(act_mat,muscle_score_vec)-act_tresh
            trig_max = np.amax(trig_score)
            trig_max_ind = np.argmax(trig_max)
            if trig_max>0.0:
                self.trig_pub.publish(header=header,
                                    frame_count=np.uint32(self.frame_count),
                                    score_ind=np.uint16(trig_max_ind),
                                    score=np.uint32(trig_max)) '''


if __name__ == '__main__':
    mtrgr = muscle_trigger()
    rospy.spin()