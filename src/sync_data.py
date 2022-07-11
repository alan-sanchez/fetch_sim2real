#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import PolygonStamped, Point32
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import message_filters

class SyncData:
    """

    """
    def __init__(self):
        """

        """
        region_sub = message_filters.Subscriber('/offset_region', PolygonStamped)
        sub_2 = message_filters.Subscriber('/joint_states', JointState)

        sync = message_filters.ApproximateTimeSynchronizer([region_sub, sub_2], queue_size=1)
        sync.registerCallback(self.callback)

        print("made it here")

    def callback(self,msg1, msg2):
        rospy.loginfo('Got a pair of messages')


if __name__ == '__main__':
    rospy.init_node('sync_data', argv=sys.argv)

    SyncData()
    rospy.spin()
