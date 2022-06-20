#!/usr/bin/env python
from __future__ import division

import sys
import actionlib
import rospy
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from best_fit import fit
from scipy import spatial
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Pose, Vector3Stamped, PointStamped
from std_msgs.msg import Header, String, Int32, Float32
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class IrradianceVectors(object):

    def __init__(self):
        print("Here first")
        # Initialize Subscribers
        self.duration_sub  = rospy.Subscriber('duration' , Float32 , self.duration_callback)

        # Initialize Publishers
        self.vector_array_pub = rospy.Publisher('vectors', numpy_msg(Floats), queue_size=10)

        # Initialize transform listener
        self.listener = tf.TransformListener()

        # Initialize self.cloud for data storage in pointcloud_data callback function
        self.cloud = None

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Initialize the end effectors point stamp data
        self.point_stamp = PointStamped()
        self.point_stamp.header = Header()
        self.point_stamp.header.frame_id = "/ee_link"
        self.point_stamp.header.stamp = rospy.Time.now()

        # Initialize waypoint_markers and all of the other feature values
        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.ARROW
        self.waypoints_marker.scale.x = 0.03
        self.waypoints_marker.scale.y = 0.01
        self.waypoints_marker.scale.z = 0.005
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 1
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 0
        self.waypoints_marker.id = 1

        # Bring in the UV light source model.
        self.model = fit(16, plotter = False)

        # Time duration set to 1 second
        self.duration = 1


    def duration_callback(self, duration):
        # Store the duration for the time of the path
        self.duration = duration.data
        print(self.duration)
        self.distance_calculator()

    def distance_calculator(self):
        # Generate 3 layers of points that will serve as the leading points of
        # the UV castRays. there will be 1 point in the center, 6 in the middle layer
        # and 12 in the outer layer. These values are in a list (n).
        # r reresents the layer distance from the center point.
        n = [1, 6, 12]
        r = [0.0, 0.025, 0.05]

        # Frome the previous defined list, n and r, we generat the points in the
        # self.circle_points function.
        circles = self.circle_points(r,n)

        # Create an empty numpy array in the 19 by 4 matrix. The first three columns
        # will be the x, y, and z directions, respectviely. The last column will be
        # the irradiance value from the model.
        self.vectors = np.empty(shape=[len(circles),4])
        print(self.vectors)

        # Rospy rates and start time.
        rate = rospy.Rate(5.0)
        start = rospy.get_time()
        iterations = 0

        while not rospy.is_shutdown():
            # self.get_matrix returns a Matrix that converts  points referencing the
            # end effector to base_link
            M = self.get_matrix(self.point_stamp)

            # self.find_ee_pose() gets the translational and rotational difference
            # from the end_effector to the base_link. Essentially, the end_effector's
            # coordinates when using the base_link tf as the reference.
            ee_trans, ee_rot = self.find_ee_pose()

            i = 0

            # For each row (a total of 19) of coordiates, we are creating a directional
            # vector.
            for e in circles:
                # Store the x,y, and z coordinates ( which is referencing the end effector)
                # in a 1 x 4 list for the matrix transformation.
                point_coord = [e[0], e[1], e[2], 1]S

                # transform the point_coord using the dot product function in numpy.
                # This will get the coordinates to reference in terms of the base_link
                # tf rather than the ee_link tf.
                transformed_point_coord = np.dot(M,point_coord)

                # Compute the direction vector from the transformed point and the
                # the end effector point.
                self.vectors[i] = [transformed_point_coord[0] - ee_trans[0],
                                   transformed_point_coord[1] - ee_trans[1],
                                   transformed_point_coord[2] - ee_trans[2],
                                   1]

                # Conditioal statement to fill in the irradiance values from our model.
                # These values will be stored in the 4th column
                if i == 0:
                    self.vectors[i][3] = self.model(r[0]*100)

                elif i > 0 and i < 6:
                    self.vectors[i][3] = self.model(r[1]*100)

                elif i >= 6:
                    self.vectors[i][3] = self.model(r[2]*100)

                # Increment for the conditional statement above.
                i+=1

            # Publish both the location of the end effector and the vectors list
            # for the accumulation modeling.
            a = np.array([ee_trans[0],ee_trans[1],ee_trans[2]], dtype=np.float32)
            b = np.array(self.vectors.ravel(), dtype=np.float32)
            self.vector_array_pub.publish(np.concatenate((a,b)))

            # Once the computation has surpassed to time duration then break from the while loop
            if (rospy.get_time() - start) > self.duration:
                break

            rate.sleep()

    def find_ee_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/ee_link',rospy.Time(0))
                return [trans,rot]
                if trans:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def get_matrix(self, point_stamp):
        while not rospy.is_shutdown():
            try:
                point_stamp = Header()
                point_stamp.frame_id = "/ee_link"
                point_stamp.stamp = rospy.Time.now()
                new_Point = self.listener.asMatrix('/base_link', point_stamp)
                return new_Point
                if new_Point:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def circle_points(self, r, n):
        circles = []
        for r, n in zip(r, n):
            t = np.linspace(0, 2*np.pi, n, endpoint=False)
            z = r * np.cos(t)
            y = r * np.sin(t)
            x = [0.3]*len(z)
            circles.append(np.c_[x, y, z])
            concatenate = np.concatenate( circles, axis=0 )
        return concatenate


if __name__=="__main__":
    rospy.init_node('irradiance_vectors',anonymous=True)
    IrradianceVectors()
    rospy.spin()
