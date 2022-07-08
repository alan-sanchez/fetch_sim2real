#!/usr/bin/env python3

# Import modules
import sys
import actionlib
import rospy
import numpy as np
import octomap

# Import message types and other pyton libraries
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point32


class Accumulator:
    """
    A class that subscribes to the UV direction vectors and publishes a matrix
    that represents a UV accumulation depth map.
    """
    def __init__(self):
        """
        Function that initializes the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        # Initialize Subscriber
        self.vector_sub = rospy.Subscriber('/vectors', numpy_msg(Floats), self.irradiance_vectors, queue_size=10)

        # Initialize Publisher

        # Create an empty list for future storage of the rays that hit.
        self.hit_list = []

        # Create and epty list for the irradiance values for the rays that hit.
        self.ir_list = []

        # Set a previous time as none. This will be used for [ADD HERE]
        self.prev_time = None

    def irradiance_vectors(self, msg):
        """
        Function that determines if they hit the depth map. This returns a distance
        value in which the inverse square law can be applied to solve the for UV accumulation.
        :param self: The self reference.
        :param vectors: Array of floats.
        """
        # This resets the prev_time variable to None after a trajectory execution
        # is complete. This is because there is a 2 second wait time before the user
        # can generate another random region to disinfect.
        if (rospy.get_time() - self.prev_time) > 2:
            self.prev_time = None

        # For the first  or reset of the prev_time variable.
        if self.prev_time == None:
            self.prev_time = rospy.get_time()

        # The end effector location and its referencing the base link.
        ee_loc = np.array([msg.data[0], msg.data[1], msg.data[2]], dtype=np.double)

        # The direction vectors and their irradiance values are set as a long list
        dir_ir_vectors = msg.data[3:]

        # end is a zero array where the location of the hit will be stored.
        end = np.array([0,0,0], dtype=np.double)

        # # Begin Timer for computed the UV dose (UV irradiance x time exposure)
        # start = rospy.get_time()

        hits = []
        irradiance = []

        # Use for loop to parse directional vector data and check if each hits
        # a location in the disinfection region.
        for j in range(int(len(dir_vectors)/4)):
            vector = np.array([v[j*4 + 0],
                               v[j*4 + 1],
                               v[j*4 + 2]], dtype=np.double)

            # Use castRay function in octree class to return a hit location
            hit = self.octree.castRay(ee_loc,
                                      vector,
                                      end,
                                      ignoreUnknownCells = True,
                                      maxRange = 1.0)

            # If there is a hit, then continue next set of computations
            if hit:

                # Append hit location, end, to a hit_list
                hits.append(end.tolist())

                # Get euclidean distance from end and end_effector tf.
                Ray_length = np.sqrt(np.sum((end-ee_loc)**2, axis=0))

                #
                dist_ratio = (0.3**2)/Ray_length**2
                time_exposure = abs(rospy.get_time() - self.prev_time)
                ir =  dir_ir_vectors[j*4 + 3]
                irradiance.append(dist_ratio * time_exposure * ir)


        for k in range(len(irradiance)):
            if hits[k] in self.hit_list:
                index = self.hit_list.index(hit_list[k])
                self.ir_list[index] = irradiance[k] + self.ir_list[index]

            else:
                self.hit_list.append(hit_list[k])
                self.ir_list.append(irradiance[k])


        self.prev_time = rospy.get_time()






if __name__=="__main__":
    rospy.init_node('castRays',anonymous=True)
    CastRays()
    rospy.spin()

    #
    #
    # if end.tolist() in hit_list:
    #     pass
    # else:
    #     hit_list.append(end.tolist())
    #
    #     Ray_length = np.sqrt(np.sum((end-ee_loc)**2, axis=0))
    #     dist_ratio = (0.3**2)/Ray_length**2
    #     time_exposure = abs(rospy.get_time() - self.prev_time)
    #     ir =  v[j*4 + 3]
    #     irradiance.append(dist_ratio * time_exposure * ir)
