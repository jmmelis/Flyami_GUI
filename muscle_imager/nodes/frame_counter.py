#!/usr/bin/env python

import rospy
import rosparam
import rospkg

from std_msgs.msg import Header, String, Bool, UInt32
from sensor_msgs.msg import Image
from muscle_imager.msg import MsgTrigger

import numpy as np

class frame_counter(object):
    
    def __init__(self):
        # initialize the node
        rospy.init_node('muscle_trigger')
        self.namespace = rospy.get_namespace()
        self.nodename = rospy.get_name().rstrip('/')

        # Subscribe to teensy output
        rospy.Subscriber('/frame_nr', String, self.new_frame_nr_callback)

        # Publish whether the fly is flying or not
        self.pubFlyFlying = rospy.Publisher(self.nodename+'/fly_flying', Bool, queue_size=1000)

        # Publish whether the high-speed cameras are downloading or not
        self.pubDownloading = rospy.Publisher(self.nodename+'/downloading', Bool, queue_size=1000)

        # Publish the frame sync count number
        self.pubFrameCount = rospy.Publisher(self.nodename+'/frame_count', UInt32, queue_size=1000)

        # Publish a trigger message
        self.pubTriggerMsg = rospy.Publisher(self.nodename+'/trigger_msg', MsgTrigger, queue_size=1)

        # Publish pulse count number
        self.pubOptoPulse = rospy.Publisher(self.nodename+'/opto_count', UInt32, queue_size=1000)

    def new_frame_nr_callback(self,msg):
        # update state publishers when a new frame message arrives
        header = Header(stamp=rospy.Time.now())
        msg_str = str(msg.data)
        msg_mode = msg_str[0]
        msg_count = msg_str[1:len(msg_str)]

        if msg_mode == 'N':
            self.pubFlyFlying.publish(False)
            self.pubDownloading.publish(False)
            self.pubFrameCount.publish(np.uint32(msg_count))
        elif msg_mode == 'F':
            self.pubFlyFlying.publish(True)
            self.pubDownloading.publish(False)
            self.pubFrameCount.publish(np.uint32(msg_count))
        elif msg_mode == 'D':
            self.pubFlyFlying.publish(True)
            self.pubDownloading.publish(True)
            self.pubFrameCount.publish(np.uint32(msg_count))
        elif msg_mode == 'T':
            self.pubFlyFlying.publish(True)
            self.pubDownloading.publish(False)
            self.pubFrameCount.publish(np.uint32(msg_count))
            self.pubTriggerMsg.publish(header=header,frame_count=np.uint32(msg_count))
        elif msg_mode == 'L':
            self.pubOptoPulse.publish(np.uint32(msg_count))

if __name__ == '__main__':
    frmcnt = frame_counter()
    rospy.spin()
