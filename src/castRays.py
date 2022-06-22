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


class CastRays:
    """
    
    """
    def __init__(self):
        # Initialize Subscribers
        self.vector_sub      = rospy.Subscriber('/vectors', numpy_msg(Floats), self.irradiance_vectors, queue_size=10)

        self.iter = 0
        self.hit_list = []
        self.ir_list = []
        self.prev_time = None

    def irradiance_vectors(self, vectors):

        origin = np.array([vectors.data[0], vectors.data[1], vectors.data[2]], dtype=np.double)
        v = vectors.data[3:]
        end = np.array([0,0,0], dtype=np.double)
        start = rospy.get_time()

        hit_list = []
        irradiance = []

        for j in range(int(len(v)/4)):
            vector = np.array([v[j*4 + 0],
                               v[j*4 + 1],
                               v[j*4 + 2]], dtype=np.double)

            hit = self.octree.castRay(origin,
                                      vector,
                                      end,
                                      ignoreUnknownCells = True,
                                      maxRange = 1.0)


            if hit:
                if self.prev_time == None:
                    self.prev_time = rospy.get_time()

                if end.tolist() in hit_list:
                    pass
                else:
                    hit_list.append(end.tolist())

                    Ray_length = np.sqrt(np.sum((end-origin)**2, axis=0))
                    dist_ratio = (0.3**2)/Ray_length**2
                    time_exposure = abs(rospy.get_time() - self.prev_time)
                    ir =  v[j*4 + 3]
                    irradiance.append(dist_ratio * time_exposure * ir)


        for k in range(len(irradiance)):
            if hit_list[k] in self.hit_list:
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
