#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import PolygonStamped, Point32
from sensor_msgs.msg import JointState
from rospy_tutorials.msg import Floats
from std_msgs.msg import Header
from fetch_sim2real.msg import HeaderArray

import message_filters

class SyncData:
    """

    """
    def __init__(self):
        """

        """
        region_sub = message_filters.Subscriber('/accumulation_map', HeaderArray)
        sub_2 = message_filters.Subscriber('/joint_states', JointState)

        sync = message_filters.ApproximateTimeSynchronizer([region_sub, sub_2], queue_size=10, slop=0.1)
        sync.registerCallback(self.callback)

        print("made it here")

    def callback(self, msg1, msg2):
        rospy.loginfo('Got a pair of messages')


if __name__ == '__main__':
    rospy.init_node('sync_data', argv=sys.argv)

    SyncData()
    rospy.spin()
