#!/usr/bin/env python3

# Import what we need
import rospy
import math
import actionlib
import numpy as np
import random
import pyvista as pv
import pickle

from scipy import spatial, stats
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point, PolygonStamped, PoseStamped, Point32
from std_msgs.msg import Header
from grid_based_sweep_coverage_path_planner import planning_animation,planning
import matplotlib.pyplot as plt



# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from boustrophedon_msgs.msg import PlanMowingPathAction, PlanMowingPathGoal, PlanMowingPathResult

class Waypoint_generator:
    """
    A class that generates waypoints within the disinfection region.
    """
    def __init__(self):
        """
        Initialize subscribers, publishers, and other message types.
        :param self: The self reference.
        """
        # Initialize Subscribers
        self.pointcloud_sub  = rospy.Subscriber("filtered_cloud", PointCloud,     self.callback_pcl,    queue_size=1)
        self.region_sub      = rospy.Subscriber("offset_region",         PolygonStamped, self.callback_region, queue_size=1)

        # Initialize Publishers
        self.waypoints_pub        = rospy.Publisher('waypoints'       , PoseArray , queue_size=1)
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker, queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup PoseArray for the waypoints.
        self.waypoints = PoseArray()
        self.waypoints.header = self.header

        # Setup Marker for the waypoints. This will be visualized in rviz
        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.ARROW
        self.waypoints_marker.scale.x = 0.03
        self.waypoints_marker.scale.y = 0.01
        self.waypoints_marker.scale.z = 0.005
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 0
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 1.0


        # Set the offset
        self.offset = 0.3

        # Set the waypoint resolution (distance between points)
        self.resolution = 0.05


    def callback_pcl(self, pcl_msg):
        """
        Callback function that parse and stores the x,y, and z coodinates of the
        PointCloud message.
        :param self: The self reference.
        :param pcl_msg: The
        """
        # Store the x, y, and z coordinates of the filtered pointcloud in separate lists
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_z = []

        # For loop to extract x, y, and z coordinates of the pointcloud
        for i in range(len(pcl_msg.points)):
            self.cloud_x.append(pcl_msg.points[i].x)
            self.cloud_y.append(pcl_msg.points[i].y)
            self.cloud_z.append(pcl_msg.points[i].z)

        # Run waypoint planner for non-planar surfaces.
        self.waypoint_planner()


    def callback_region(self, msg):
        # Store the x and y positions of polygon message in separate lists
        self.Polygon_X = []
        self.Polygon_Y = []

        # For loop to extract x and y coordinates of the polygon
        for vertices in msg.polygon.points:
            self.Polygon_X.append(vertices.x)
            self.Polygon_Y.append(vertices.y)

        # Append first x and y coordinates to close the polygon loop
        self.Polygon_X.append(self.Polygon_X[0])
        self.Polygon_Y.append(self.Polygon_Y[0])


    def waypoint_planner(self):
        # Acquire the planned x an y values from the planning function in the
        # grid_based_sweep_coverage_path_planner python script.
        px,py = planning(self.Polygon_X, self.Polygon_Y, self.resolution)

        # Create space-partition data strucutre (KD tree) for filtered cloud
        tree = spatial.KDTree(np.c_[self.cloud_x,self.cloud_y])

        # Delete previous ARROW markers and publish the empty data structure
        self.waypoints_marker.action = Marker.DELETEALL
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        # Set marker action to add for new ARROW markers
        self.waypoints_marker.action = Marker.ADD


        # Create lists for waypoints and waypoint markers that will be publish
        poses = []

        for i in range(len(px)):
            # # Get index values closest neighbors of the planned x and y values
            # # in the KDtree
            # closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.025)
            #
            # # If there are no closest neighbors, then continue to next iteration
            # # of the for loop
            # if len(closest_points_ii ) == 0:
            #     continue
            #
            # self.z_val = []
            # # append the z values of the point cloud of the indexed values.
            # for j in closest_points_ii:
            #     self.z_val.append(self.cloud_z[j])
            #
            #
            # # Find the index of the max z value from the closest neighbors point cloud
            # index = closest_points_ii[self.z_val.index(max(self.z_val))]


            # Include characteristics of a pose
            p = Pose()
            p.position.x = px[i]#self.cloud_x[index]
            p.position.y = py[i]#self.cloud_y[index]
            p.position.z = self.cloud_z[i] + self.offset#self.cloud_z[index] + self.offset
            p.orientation.x = 0
            p.orientation.y = .7070
            p.orientation.z = 0
            p.orientation.w = .7070
            poses.append(p)

            # Create new marker id and pose to be published
            self.waypoints_marker.id = i
            self.waypoints_marker.pose = p
            self.waypoints_marker_pub.publish(self.waypoints_marker)
            rospy.sleep(0.005)


        # assisn poses to the PoseArray, self,waypoints.
        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Clear out cloud data for new updated data
        del self.cloud_x[:], self.cloud_y[:], self.cloud_z[:]



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')
    Waypoint_generator()
    rospy.spin()
