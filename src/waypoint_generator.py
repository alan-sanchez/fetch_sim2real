#!/usr/bin/env python3

# Import what we need
import rospy
import math
import actionlib
import numpy as np
import random
import pyvista as pv

from scipy import spatial, stats

from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point, PolygonStamped, PoseStamped, Point32
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg

# We need the MoveBaseAction and MoveBaseGoal from the move_base_msgs package.
from boustrophedon_msgs.msg import PlanMowingPathAction, PlanMowingPathGoal, PlanMowingPathResult

class Waypoint_generator:
    def __init__(self):
        # Initialize Subscribers
        self.pointcloud_sub  = rospy.Subscriber("filtered_cloud", PointCloud, self.data_pointcloud, queue_size=1)

        # Set up a client for the basic nav, receives the user_defined Action
        self.coverage_client = actionlib.SimpleActionClient('plan_path', PlanMowingPathAction)
        rospy.loginfo('Waiting to connect with server')
        self.coverage_client.wait_for_server()
        rospy.loginfo('Made contact with move server')

        # Initialize Publishers
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker, queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Setup PolygonStamped for the 2D subplane
        self.region = PolygonStamped()
        self.region.header = self.header

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

        self.Polygon_X = [0.65, 0.85, 0.85, 0.65, 0.65]
        self.Polygon_Y = [0.45, 0.45, 0.10, 0.10, 0.45]

        # Set the offset
        # self.offset = .3


    def data_pointcloud(self, pcl_msg):
        # Store the x, y, and z coordinates of the filtered pointcloud in separate lists
        self.cloud_x = []
        self.cloud_y = []
        self.cloud_z = []

        # For loop to extract x, y, and z coordinates of the pointcloud
        for i in range(len(pointcloud_msg.points)):
            self.cloud_x.append(pointcloud_msg.points[i].x)
            self.cloud_y.append(pointcloud_msg.points[i].y)
            self.cloud_z.append(pointcloud_msg.points[i].z)

        # Run waypoint planner for non-planar surfaces.
        self.waypoint_planner()

    def assign_goal(self):
        # Create the x,y, and z coordinates of the disinfection region verticies
        verticies = [[0.65, 0.45, 0.63], [0.85, 0.45, 0.63], [0.85, 0.10,0.63], [0.65, 0.10, 0.63]]

        for i in range(len(verticies)):
            self.region.polygon.points.append(Point32(verticies[i][0], verticies[i][1], verticies[i][2]))
        #
        #
        start_pose = PoseStamped()
        start_pose.header = self.header
        start_pose.pose.position.x    = self.region.polygon.points[0].x
        start_pose.pose.position.y    = self.region.polygon.points[0].y
        start_pose.pose.position.z    = self.region.polygon.points[0].z
        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 1.0

        goal = PlanMowingPathGoal(property=self.region,robot_position=start_pose)
        self.coverage_client.send_goal(goal, done_cb=self.done_callback)
        self.coverage_client.wait_for_result()


    def done_callback(self, status, result):
        print(len(result.plan.points))
        # Delete previous ARROW markers and publish the empty data structure
        self.waypoints_marker.action = Marker.DELETEALL
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        # Set marker action to add for new ARROW markers
        self.waypoints_marker.action = Marker.ADD

        poses = []
        marker_list = []
        # print("")
        # print(result.plan.points[0].point.x)
        for i in range(len(result.plan.points)):
            p = Pose()
            p.position.x = result.plan.points[i].point.x
            p.position.y = result.plan.points[i].point.y
            p.position.z = result.plan.points[i].point.z
            p.orientation.x = 0
            p.orientation.y = .707
            p.orientation.z = 0
            p.orientation.w = .707
            poses.append(p)
            print(p)
            # Create new marker id and pose to be published
            self.waypoints_marker.id = i
            self.waypoints_marker.pose = p
            self.waypoints_marker_pub.publish(self.waypoints_marker)
        print("made it here")
        print("")
        print('')
        print("")
        #     marker_list.append(Point(p.position.x, p.position.y, p.position.z))
        #
        # self.waypoints_marker.points = marker_list
        # self.waypoints_marker_pub.publish(self.waypoints_marker)
        # print('published something')
        # for i in range(len(px)):
        #             # Get index values closest neighbors of the planned x and y values
        #             # in the KDtree
        #             closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.025)
        #
        #             # If there are no closest neighbors, then continue to next iteration
        #             # of the for loop
        #             if len(closest_points_ii ) == 0:
        #                 continue
        #
        #             self.z_val = []
        #             # append the z values of the point cloud of the indexed values.
        #             for j in closest_points_ii:
        #                 self.z_val.append(self.cloud_z[j])
        #
        #             # Remove any z outliers of the stored data.
        #             self.remove_outliers()
        #
        #             # Find the index of the max z value from the closest neighbors point cloud
        #             index = closest_points_ii[self.z_val.index(max(self.z_val))]
        #
        #
        #             # Include characteristics of a pose
        #             p = Pose()
        #             p.position.x = self.cloud_x[index]
        #             p.position.y = self.cloud_y[index]
        #             p.position.z = self.cloud_z[index] + self.offset
        #             p.orientation.x = 0
        #             p.orientation.y = .7070
        #             p.orientation.z = 0
        #             p.orientation.w = .7070
        #             poses.append(p)
        #
        #             # Create new marker id and pose to be published
        #             self.waypoints_marker.id = i
        #             self.waypoints_marker.pose = p
        #             self.waypoints_marker_pub.publish(self.waypoints_marker)






if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')
    Waypoint_generator()
    # gen.assign_goal()
    rospy.spin()
