#!/usr/bin/env python

import rospy
import sys
import message_filters
import pandas as pd
# import csv

from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, Float32
from sensor_msgs.msg import PointCloud
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from fetch_sim2real.msg import HeaderArray
from csv import writer, DictWriter



class SyncData:
    """

    """
    def __init__(self):
        """

        """
        # Initialize subscribers
        self.waypoints_sub  = rospy.Subscriber('/waypoints',      PoseArray,         self.callback_waypoints)
        self.pointcloud_sub = rospy.Subscriber('/filtered_cloud', PointCloud,        self.callback_pointcloud)
        self.velocities_sub = rospy.Subscriber('/velocities',     numpy_msg(Floats), self.callback_velocities)
        self.stop_sub       = rospy.Subscriber('/stop',           String,            self.callback_stop_command)


        self.accumulation_map_sub = message_filters.Subscriber('/accumulation_map', HeaderArray)
        self.joint_states_sub     = message_filters.Subscriber('/joint_states',     JointState)
        self.ee_pose_sub          = message_filters.Subscriber('/ee_pose',          PoseStamped)

        sync = message_filters.ApproximateTimeSynchronizer([self.accumulation_map_sub,
                                                            self.joint_states_sub,
                                                            self.ee_pose_sub],
                                                            queue_size=10,
                                                            slop=0.1)
        sync.registerCallback(self.callback_sync)

        # Initialize filtered_cloud list
        self.filtered_cloud = []

        # Initialize waypoints list
        self.waypoints = []

        # Initialize velocities List
        self.velocities = []

        # Initiailze command String
        self.command = "start"


    def callback_pointcloud(self, msg):
        """
        Function that stores the filtered point cloud and create new octree for
        castRay calculations.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        # Store the filtered point cloud
        self.filtered_cloud = msg

    def callback_waypoints(self,msg):
        """
        Function that stores the PoseArray messages.
        :param self: The self reference.
        :param msg: The PoseArray message type.
        """
        self.waypoints = msg

    def callback_velocities(self,msg):
        """
        Function that stores the Float messages.
        :param self: The self reference.
        :param msg: The Float array message.
        """
        self.velocities = msg

    def callback_stop_command(self,msg):
        """
        Function that stores the String message.
        :param self: The self reference.
        :param msg: The String message.
        """
        self.command = msg
        ee_loc_header = ['Position_x', 'Position_y','Position_z', 'Quaternion_x', 'Quaternion_y', 'Quaternion_z', 'Quaternion_w']
        # with open("CSVFILE.csv", 'w') as file:
        #     dw = DictWriter(file, delimiter=',',
        #                     fieldnames=ee_loc_header)
        #     dw.writeheader()




    def callback_sync(self, msg1, msg2, msg3):
        ee_loc = [msg3.pose.position.x, msg3.pose.position.x, msg3.pose.position.x, msg3.pose.orientation.x, msg3.pose.orientation.y, msg3.pose.orientation.z, msg3.pose.orientation.w]
        #
        # df = pd.DataFrame(ee_loc, index=['ee_pose'],columns = ee_loc_header)
        #
        # df.to_csv('Test.csv', mode='a', index=False, header=False)

        with open ('CSVFILE.csv', 'a+') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(ee_loc)

        rospy.loginfo('Got three messages')




if __name__ == '__main__':
    rospy.init_node('sync_data', argv=sys.argv)

    SyncData()
    rospy.spin()
