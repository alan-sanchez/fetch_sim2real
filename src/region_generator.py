#!/usr/bin/env python

# Import python modules
import numpy as np
import random
import rospy
import sys

# Import message types and other python libraries
from scipy.spatial import ConvexHull
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
from shapely.geometry.polygon import LinearRing

class Region:
    """
    A class that randomly generates a polygon region for the Fetch to disinfect.
    """
    def __init__(self):
        """
        Initializes publishers and other message types.
        :param self: The self reference.
        """
        # Initialize Publishers
        self.region_pub = rospy.Publisher('region', PolygonStamped, queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup self.region as a PolygonStamped message type
        self.region = PolygonStamped()
        self.region.header = self.header

    def convex_hull(self):
        """
        Function that uses that convex hull method to create a disinfection region.
        :param self: The self reference.

        :publishes self.region: The PolygonStamped Message
        """

        # Create an empty list to store 10 random x and y coordinates
        coord = []
        for i in range(10):
            x = random.uniform(0.6, 0.9)
            y = random.uniform(0.55, -0.1)
            coord.append([x,y])

        # Run convex hull function on the random 2D plane coordinates
        hull = ConvexHull(coord)

        # Reset the polygon.points for the self.region data type
        self.region.polygon.points = []
        line = []

        # Store the hull vertices to the polygon.points
        for e in hull.vertices:
            self.region.polygon.points.append(Point32(coord[e][0], coord[e][1], 0.63))
            line.append([coord[e][0], coord[e][1]])

        # Publish polygon regions
        self.region_pub.publish(self.region)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('region_generator')

    # Declare object from Region class
    region = Region()

    # Set rospy rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Wait for user input before generating new disinfection region
        raw_input()

        # Run the convex hull function
        region.convex_hull()
        rate.sleep()
