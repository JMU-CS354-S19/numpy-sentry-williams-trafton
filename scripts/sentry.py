#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Author: Chris Williams, Elena Trafton 
Version:
"""

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class SentryNode(object):
    """Monitor a vertical scan through the depth map and create an
    audible signal if the change exceeds a threshold.

    Subscribes:
         /camera/depth_registered/image
       
    Publishes:
        /mobile_base/commands/sound

    """
    

    def __init__(self):
        """ Set up the Sentry node. """
        rospy.init_node('sentry')
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
        rospy.spin()
        self.d = None
        self.prev_slice = None
        self.avg = None

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """

        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        # YOUR CODE HERE.
        # HELPER METHODS ARE GOOD.
        v_slice = depth[:,len(depth)/2]
        v_slice = v_slice[~np.isnan(v_slice)]
        self.depth_update(v_slice)
        if (self.d / self.avg) > .5
            self.signal_intruder()


    def depth_update(self, some_slice):
        if self.prev_slice == None:
            self.prev_slice = some_slice
        elif self.d == None:
            self.d = self.prev_slice - some_slice
            self.prev_slice = some_slice
        else:
            self.d += self.prev_slice - some_slice
            self.prev_slice = some_slice
        self.update_avg()

    def update_avg(self):
        alpha = .7
        if self.average == None:
            self.average = self.prev_slice
        else:
            self.average = self.average * alpha + self.d * (1-alpha)

    def signal_intruder(self):
        pub = rospy.Publisher('/mobile_base/commands/sound')
        pub.publish(4)
        
        


if __name__ == "__main__":
    SentryNode()
