#!/usr/bin/env python

# Import modules
import rospy
import numpy as np
import tf

# Import message types and other python libraries
from best_fit import fit
from geometry_msgs.msg import Vector3Stamped, PointStamped, PoseStamped
from std_msgs.msg import Header, String, Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class IrradianceVectors(object):
    """
    A class that publishes the direction and irridance values of the UV vectors
    whose origins are from the end effector.
    """
    def __init__(self):
        """
        A function that initialises  the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        # Initialize Subscribers
        self.start_sub = rospy.Subscriber('start', String, self.ir_vec_calculator)
        self.stop_sub  = rospy.Subscriber('stop',  String, self.callback_stop_command)

        # Initialize Publishers
        self.vector_array_pub = rospy.Publisher('vectors', numpy_msg(Floats), queue_size=10)
        self.ee_pose_pub      = rospy.Publisher('ee_pose', PoseStamped,       queue_size=10)

        # Initialize transform listener
        self.listener = tf.TransformListener()

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Initialize the end effector PoseStamped message type. Used for data storage
        self.ee_pose_stamped = PoseStamped()
        self.ee_pose_stamped.header = self.header

        # Bring in the UV light source model as an equation
        self.eqn_model = fit(16, plotter = False)

    def callback_stop_command(self, msg):
        """
        A callback function that stores a String message that stops this node from
        publishing irridiance vectors.
        :param self: The self reference.
        :param msg: The String message type.
        """
        self.command = msg.data

    def ir_vec_calculator(self, msg):
        """
        A callback function that generates vectors with their given irradiance values
        and publishes those values, along with the end effector location.
        :param self: The self reference.
        :param msg: The String message type.
        """
        # Set self.command to incoming String message
        self.command = msg.data

        # Generate 3 circle layers of points that will serve as the LEADING points of
        # the UV castRays. there will be 1 point in the center, 6 in the middle layer
        # and 12 in the outer layer. These values are defined in a list named "n".
        # "r" reresents the layer distance from the center point in centimeters
        n = [1, 6, 12]
        r = [0.0, 2.5, 5.0]

        # From the previous defined list, n and r, we generate the points in the
        # self.circle_points function
        circles = self.circle_points(r,n)

        # Create an empty numpy array, size of a 19 by 4 matrix. The first three columns
        # will be the x, y, and z vector directions, respectviely. The last column will be
        # the irradiance value of that directional vector, which is acquired from the model
        self.vectors = np.empty(shape=[len(circles),4])

        # Rospy rates and start time.
        rate = rospy.Rate(20.0)
        start = rospy.get_time()
        iterations = 0

        while self.command == "start":
            # self.get_matrix returns a transformation matrix that converts coordinates
            # in the ee_link frame to the base_link frame
            M = self.get_matrix()

            # self.find_ee_pose() gets the translational and rotational difference
            # from the ee_link to the base_link. Essentially, the ee_link's
            # coordinates in reference to the base_link tf.
            ee_trans, ee_rot = self.find_ee_pose()

            i = 0

            # For each row (a total of 19) of coordiates, we create a directional vector
            for e in circles:
                # Store the x,y, and z circle coordinates ( which is referencing
                # the end effector) in a 1 x 4 list for the matrix transformation
                circle_coord = [e[0], e[1], e[2], 1]

                # transform the circle_coord using the transformation matrix and the
                # dot product function in numpy. This will get the coordinates in relation
                # to the base_link
                transformed_circle_coord = np.dot(M,circle_coord)

                # Compute the direction vector from the coordinates of the
                # transformed point and the end effector point
                self.vectors[i] = [transformed_circle_coord[0] - ee_trans[0],
                                   transformed_circle_coord[1] - ee_trans[1],
                                   transformed_circle_coord[2] - ee_trans[2],
                                   1]

                # Conditional statement to fill in the irradiance values from our model.
                # These values will be stored in the 4th column of the vectors dataset
                if i == 0:
                    self.vectors[i][3] = self.eqn_model(r[0])

                elif i > 0 and i < 6:
                    self.vectors[i][3] = self.eqn_model(r[1])

                elif i >= 6:
                    self.vectors[i][3] = self.eqn_model(r[2])

                # Increment for the conditional statement above
                i+=1

            # Publish both the location of the end effector and the vectors list
            # in a single array.
            ee_pose = np.array([ee_trans[0],ee_trans[1],ee_trans[2]], dtype=np.float32)
            dir_ir_vector = np.array(self.vectors.ravel(), dtype=np.float32)
            self.vector_array_pub.publish(np.concatenate((ee_pose, dir_ir_vector)))

            # Publish PoseStamped of the end effector
            self.ee_pose_stamped.header.stamp = rospy.Time.now()
            self.ee_pose_stamped.pose.position.x = ee_trans[0]
            self.ee_pose_stamped.pose.position.y = ee_trans[1]
            self.ee_pose_stamped.pose.position.z = ee_trans[2]
            self.ee_pose_stamped.pose.orientation.x = ee_rot[0]
            self.ee_pose_stamped.pose.orientation.y = ee_rot[1]
            self.ee_pose_stamped.pose.orientation.z = ee_rot[2]
            self.ee_pose_stamped.pose.orientation.w = ee_rot[3]
            self.ee_pose_pub.publish(self.ee_pose_stamped)

            rate.sleep()

    def find_ee_pose(self):
        """
        Function that finds the pose of the ee_link relative to the base_link frame
        :param self: The self reference.

        :return [trans, rot]: The Pose message type.
        """
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/ee_link',rospy.Time(0))
                return [trans,rot]
                if trans:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def get_matrix(self):
        """
        Function that returns a transformation matrix from the ee_link to
        the target frame, the base_link.
        :param self: The self Reference.

        :return transform_matrix: The transformation matrix.
        """
        while not rospy.is_shutdown():
            try:
                ee_header = Header()
                ee_header.frame_id = "/ee_link"
                ee_header.stamp = rospy.Time.now()
                transform_matrix = self.listener.asMatrix('/base_link', ee_header)
                return transform_matrix
                if transform_matrix:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def circle_points(self, r, n):
        """
        Function that generates the head positions of the CastRays.
        :param self: The self reference.
        :param r: Float value.
        :param n: Float value.

        :return concatenate: Python list.
        """
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
    # Initialize irradiance_vectors node
    rospy.init_node('irradiance_vectors',anonymous=True)

    # Instantiate the IrradianceVectors class
    IrradianceVectors()
    rospy.spin()
