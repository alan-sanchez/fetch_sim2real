#!/usr/bin/env python

# Import modules
import rospy
import numpy as np
import ctypes
import struct
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32, PolygonStamped
from std_msgs.msg import Header
from shapely import geometry

class pcl_filter:
    """
    A class that takes in a PointCloud2 message and transfers it into a
    pointcloud message. Then the pointcloud message is filtered where only the
    points inside the disinfectin region are considered.
    """
    def __init__(self):
        """
        Function that initializes the subsribers, publishers, and other variables.
        :param self: The self reference.
        """
        # Flag for storiing the octree map
        self.flag = 0

        # Initialize Subscribers
        self.pointcloud2_sub = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2,    self.callback_pcl2, queue_size=1)
        self.region_sub      = rospy.Subscriber("/region",                      PolygonStamped, self.pcl_filter,    queue_size=1)


        # Initialize PointCloud Publisher
        self.pointcloud_pub = rospy.Publisher("/filtered_cloud", PointCloud, queue_size=1)

        # Initialize filtered_cloud as a PointCloud message type
        self.filtered_cloud = PointCloud()
        self.filtered_cloud.header = Header()
        self.filtered_cloud.header.stamp = rospy.Time.now()
        self.filtered_cloud.header.frame_id = '/base_link'

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.pcl2_cloud = None

        # Intialize tf.Transformlister
        self.listener = tf.TransformListener()

    def callback_pcl2(self,msg):
        """
        Callback function that stores the pointcloud2 message.
        :param self: The self reference.
        :param msg: The Pointcloud2 message type.
        """
        # Use flag condition so we don't continuously store the same pointcloud data
        if self.flag == 0:
            rospy.loginfo("received pointcloud")
            # Store pointcloud2 data
            self.pcl2_cloud = msg
            self.flag = 1

    def pcl_filter(self, polygon):
        """
        Callback function that stores the pointcloud2 message.
        :param self: The self reference.
        :param polygon: The PolygonStamped message.

        :publishes self.filtered_cloud: Filtered PointCloud message.
        """
        # Initialize a new point cloud message type to store position data.
        pcl_cloud = PointCloud()
        pcl_cloud.header = self.pcl2_cloud.header

        # For loop to extract pointcloud2 data into a list of x,y,z, and
        # store it in a pointcloud message (pcl_cloud)
        for data in pc2.read_points(self.pcl2_cloud, skip_nans=True):
            pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

        # Transform the pointcloud message to reference the base_link
        transformed_cloud = self.transform_pointcloud(pcl_cloud)

        # Intialize region as a list
        region = []
        # Run forloop to store polygon vertices in list named region
        for vertices in polygon.polygon.points:
            region.append([vertices.x,vertices.y])


        # Set self.filtered_cloud.points as a list
        self.filtered_cloud.points=[]

        # Use for loop to extract x and y points from the transformed cloud
        for points in transformed_cloud.points: #self.pcl_data:
            X = points.x
            Y = points.y

            # Check to see if the x and y points are in the polygon region. If so
            # then append the x,y, and z points to the filtered cloud.
            line = geometry.LineString(region)
            point = geometry.Point(X, Y)
            polygon = geometry.Polygon(line)
            if polygon.contains(point) == True:

                self.filtered_cloud.points.append(Point32(points.x,points.y,points.z))

        # Publish filtered point_cloud data
        self.pointcloud_pub.publish(self.filtered_cloud)

    def transform_pointcloud(self,pcl_cloud):
        """
        Function that stores the pointcloud2 message.
        :param self: The self reference.
        :param pcl_cloud: The PointCloud message.

        :returns new_cloud: PointCloud message.
        """
        pcl_cloud.header.stamp=rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                # run the transformPointCloud() function to change the referene frame
                # to the base_link
                new_cloud = self.listener.transformPointCloud("/base_link" ,pcl_cloud)
                return new_cloud
                if new_cloud:
                    break
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                pass

if __name__=="__main__":
    # Intiailze the pcl filter node
    rospy.init_node('pcl_filter',anonymous=True)

    # Instantiate the PCL filter class
    pcl_filter()
    rospy.spin()
