#!/usr/bin/env python3

# Import what we need
import rospy
import math
import numpy as np
import pyvista as pv

# Import message types and other python libraries.
from scipy import spatial
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose,PoseArray, Point, PolygonStamped, PoseStamped, Point32
from std_msgs.msg import Header
from grid_based_sweep_coverage_path_planner import planning_animation,planning

class WaypointGenerator:
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
        self.region_sub      = rospy.Subscriber("offset_region",  PolygonStamped, self.callback_region, queue_size=1)

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

        # Set the offset from the surface in the z direction
        self.offset = 0.3

        # Set the waypoint resolution (distance between each waypoints)
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
        self.waypoint_generator()


    def callback_region(self, msg):
        """
        Callback function that parses the PolygonStamped message type to two
        separate lists of x and y positions.
        :param self: The self reference.
        :param msg: The PolygonStamped message type.
        """
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


    def waypoint_generator(self):
        """
        A function that generates a lawn mower waypoint path from the polygon's
        x and y coordinates. This function also publishes the waypoints as a
        PoseArray message type.
        :param self: The self reference.
        """
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
        # marker_list = []

########################## Test Case 1: Fixed height ###########################
        for i in range(len(px)):

            # Include characteristics of a pose
            p = Pose()
            p.position.x = px[i]#self.cloud_x[index]
            p.position.y = py[i]#self.cloud_y[index]
            p.position.z = max(self.cloud_z) + self.offset#self.cloud_z[index] + self.offset
            p.orientation.x = 0
            p.orientation.y = .7070
            p.orientation.z = 0
            p.orientation.w = .7070
            poses.append(p)

            # Create new marker id and pose to be published
            self.waypoints_marker.id = i
            self.waypoints_marker.pose = p
            self.waypoints_marker_pub.publish(self.waypoints_marker)
            rospy.sleep(0.01)

################ Test Case 3: Offset distance from point normal ################

    # # Pass the pointcloud points to the PolyData constructor. Then run the
    # # delaunay_2d triangulation function on the PolyData
    # cloud = pv.PolyData(np.c_[self.cloud_x, self.cloud_y, self.cloud_z])
    # surf = cloud.delaunay_2d()

    #     for i in range(len(px)):
    #         # Get index values closest neighbors of the planned 2D x and y value
    #         # in the KDtree
    #         closest_points_ii = tree.query_ball_point([px[i],py[i]], 0.05)
    #
    #         # If there are no closest neighbors, then continue to next iteration
    #         # of the for loop
    #         if len(closest_points_ii ) == 0:
    #             continue
    #
    #         # Create empty list for the z (height) values of the neighbors
    #         self.z_val = []
    #         # append the z values of the point cloud of the indexed values.
    #         for j in closest_points_ii:
    #             self.z_val.append(self.cloud_z[j])
    #
    #
    #         # Find the index of the max z value from the closest neighbors
    #         index = closest_points_ii[self.z_val.index(max(self.z_val))]
    #
    #         # compute the point_normal angles
    #         alpha = np.arccos(surf.point_normals[index][0])
    #         gamma = np.arccos(surf.point_normals[index][1])
    #         beta  = np.arccos(surf.point_normals[index][2])
    #         # alpha = math.atan2(surf.points[index][2],surf.points[index][1])
    #         # gamma = math.atan2(surf.points[index][0],surf.points[index][2])
    #         # beta = math.atan2(surf.points[index][1],surf.points[index][0])
    #
    #         # print(surf.points[index])
    #         # print(self.cloud_x[index], self.cloud_y[index], self.cloud_z[index])
    #
    #         # Run rotation matrix of the three rotation angles
    #         mat = self.rotation_matrix(alpha,gamma,beta)
    #         r = R.from_rotvec([[0    , 0    , alpha],
    #                            [0    , gamma, 0    ],
    #                            [0    , 0    , beta]])
    #         # print("Scipy computation:")
    #         # print(r.as_matrix())
    #         # Obtain Quaternion values from rotational matrix
    #         m = R.from_matrix(mat)
    #         q=m.as_quat()
    #
    #         # print(surf.point_normals[index])
    #
    #         # project new point
    #         flipped_normal = [-surf.point_normals[index][0], -surf.point_normals[index][1], -surf.point_normals[index][2]]
    #         unit_vector = flipped_normal/np.linalg.norm(flipped_normal)
    #         offset = self.offset * unit_vector
    #         # print(unit_vector)
    #
    #         # Include characteristics of a pose
    #         p = Pose()
    #         p.position.x = self.cloud_x[index]# + offset[0]
    #         p.position.y = self.cloud_y[index]# + offset[1]
    #         p.position.z = self.cloud_z[index] + offset[2]
    #         p.orientation.x = q[0]
    #         p.orientation.y = q[1]
    #         p.orientation.z = q[2]
    #         p.orientation.w = -q[3]
    #         poses.append(p)
    #
    #         # Create new marker id and pose to be published
    #         self.waypoints_marker.id = i
    #         self.waypoints_marker.pose = p
    #         self.waypoints_marker.header.stamp = rospy.Time.now()
    #         self.waypoints_marker_pub.publish(self.waypoints_marker)

        # assisn poses to the PoseArray, self,waypoints.
        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Clear out cloud data for new updated data
        del self.cloud_x[:], self.cloud_y[:], self.cloud_z[:]



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')

    # Instantiate the WaypointGenerator class
    WaypointGenerator()
    rospy.spin()
