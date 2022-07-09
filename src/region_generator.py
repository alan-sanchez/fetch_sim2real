#!/usr/bin/env python

# Import python modules
import numpy as np
import random
import rospy
import sys

# Import message types and other python libraries.
from scipy.spatial import ConvexHull
from std_msgs.msg import Header
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
from shapely.geometry.polygon import LinearRing


class Region:
    """
    A class that radomly generates a polygon region for the Fetch to disinfect.
    """
    def __init__(self):
        """
		Initializes publishers and other message types.
		:param self: The self reference.
		"""
        # Initialize Publishers
        self.region_pub = rospy.Publisher('region', PolygonStamped, queue_size=1)
        self.offset_region_pub = rospy.Publisher('offset_region', PolygonStamped, queue_size=1)


        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup self.region as a PolygonStamped message type
        self.region = PolygonStamped()
        self.region.header = self.header

        # Setup self.offset_region as a PolygonStamped message type. this is to
        # help visualize how the UV exposure may exceed the limits of the
        # disinfection region.
        self.offset_region = PolygonStamped()
        self.offset_region.header = self.header

    def convex_hull(self):
        """
		Function that uses that convex hull method to create region.
		:param self: The self reference.

        :publishes self.region: The PolygonStamped Message
		"""

        # Create an empty list to stor random points
        coord = []
        for i in range(10):
            x = random.uniform(0.6, 0.9)
            y = random.uniform(0.55, -0.1)
            coord.append([x,y])

        # Run convex hull function on the random 2D sub-plane coordinates.
        hull = ConvexHull(coord)

        # Reset the polygon.points for both region and offset_region lists.
        self.region.polygon.points = []
        self.offset_region.polygon.points = []
        line = []

        # Store the hull vertices to the polygon.points
        for e in hull.vertices:
            self.region.polygon.points.append(Point32(coord[e][0], coord[e][1], 0.63))
            line.append([coord[e][0], coord[e][1]])

        # Run offset LinearRing for offset polygon.
        poly_line = LinearRing(line)
        poly_offset = poly_line.parallel_offset(0.025,resolution=16,
                                                join_style=2,
                                                mitre_limit=1)

        # Create offset for visualization in RViz
        offset_x,offset_y = poly_offset.xy

        # Append the offset's x and y coordinates using a forloop
        for i in range(len(offset_x)):
            self.offset_region.polygon.points.append(Point32(offset_x[i],offset_y[i], 0.63))


        # Publish polygon regions
        self.region_pub.publish(self.region)
        self.offset_region_pub.publish(self.offset_region)




if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('region')
    region = Region()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        raw_input()
        region.convex_hull()
        rate.sleep()
