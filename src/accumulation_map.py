#!/usr/bin/env python3

# Import modules
import rospy
import numpy as np
import octomap

# Import message types and other pyton libraries
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point32
from fetch_sim2real.msg import HeaderArray

class AccumulationMap:
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
        self.vector_sub      = rospy.Subscriber('/vectors',        numpy_msg(Floats), self.irradiance_vectors)
        self.pointcloud_sub  = rospy.Subscriber('/filtered_cloud', PointCloud,        self.callback_pointcloud)
        self.stop_sub        = rospy.Subscriber('/stop',           String,            self.callback_stop_command)

        # Initialize Publisher
        self.acc_map_pub  = rospy.Publisher('/accumulation_map', HeaderArray, queue_size=10)

        # Create an empty list for future storage of the rays that hit
        self.hit_list = []

        # Create a list of the accumulation map
        self.accumulation_map = HeaderArray()
        self.accumulation_map.header = Header()
        self.accumulation_map.header.frame_id = "/base_link"

        self.acc_map_list = []

        # The required UV Dose for a UV rate constant of 0.0867 m^2/J is 132.8 (J/m^2)
        self.required_dose = 132.8

        # Set a previous time as none. This will be used for [ADD HERE]
        self.prev_time = rospy.get_time()

        # Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.05
        self.octree = octomap.OcTree(self.resolution)

        # Initialize filtered_cloud variable
        self.filtered_cloud = None

        # Initialize command
        self.command = None

    def callback_pointcloud(self, msg):
        """
        Function that stores the filtered point cloud and create new octree for
        castRay calculations.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        # Store the filtered point cloud
        self.filtered_cloud = msg

        # Parse the filtered cloud's points as a np.array. This action is required
        # to pass as an agrument in the insertPointCloud() function.
        points = np.empty(shape=[len(self.filtered_cloud.points),3])
        for i in range(len(self.filtered_cloud.points)):
            points[i] = [self.filtered_cloud.points[i].x,
                         self.filtered_cloud.points[i].y,
                         self.filtered_cloud.points[i].z]

        self.octree.insertPointCloud(pointcloud = points, origin = np.array([0, 0, 0], dtype=float))

    def callback_stop_command(self, msg):
        """
        A callback function that stores a String message that stops the node from
        publishing irridance vectors.
        :param self: The self reference.
        :param msg: The String message type.
        """
        # self.command = msg
        del self.hit_list[:], self.acc_map_list[:]

    def irradiance_vectors(self, msg):
        """
        Function that determines if they hit the depth map. This returns a distance
        value in which the inverse square law can be applied to solve the for UV accumulation.
        :param self: The self reference.
        :param vectors: Array of floats.
        """
        # This resets the prev_time variable to None after a trajectory execution
        # is complete. This is because there is a 2 second wait time before the user
        # can generate another random region to disinfect
        if (rospy.get_time() - self.prev_time) > 2.0:
            self.prev_time = None

        # A conditional statement for the first  or reset of the prev_time variable.
        # If true, then the self.prev_time is defined and the the hit list and accumulation
        # map lists are cleared out
        if self.prev_time == None:
            self.prev_time = rospy.get_time()
            del self.hit_list[:], self.acc_map_list[:]


        # Extract the end effector location data. NOTE: These coordinates are
        # referencing the base_link transform frame
        ee_loc = np.array([msg.data[0], msg.data[1], msg.data[2]], dtype=np.double)

        # Extract the direction vectors and their irradiance values 
        dir_ir_vectors = msg.data[3:]

        # end is initializes and set as a zero array where the location of the hit will be stored
        end = np.array([0,0,0], dtype=np.double)

        # Compute the UV time exposure 
        time_exposure = abs(rospy.get_time() - self.prev_time)

        # Use for loop to parse directional vector data and check if each vector
        # hits a occupancy cube in the 3D disinfection grid map.
        for j in range(int(len(dir_ir_vectors)/4)):

            vector = np.array([dir_ir_vectors[j*4 + 0],
                               dir_ir_vectors[j*4 + 1],
                               dir_ir_vectors[j*4 + 2]], dtype=np.double)

            # Use the castRay function in octree class to return a hit location
            hit = self.octree.castRay(ee_loc,
                                      vector,
                                      end,
                                      ignoreUnknownCells = True,
                                      maxRange = 1.0)

            # If there is a hit, then continue next set of computations
            if hit:
                # Get euclidean distance from hit location and ee_link position
                Ray_length = np.sqrt(np.sum((end-ee_loc)**2, axis=0))

                # Use the distance ratio equation between the castRay and our 
                # measurements from our model (0.3m)
                dist_ratio = (0.3**2)/(Ray_length**2)

                # Extract the irradiance value for the parsed directional vector
                ir =  dir_ir_vectors[j*4 + 3]

                # Compute the UV dose for each vector
                dose = dist_ratio * time_exposure * ir

                # If end (which was converted to a list using the `tolist()` function) is
                # in self.hit_list, then update self.acc_map_list 
                if end.tolist() in self.hit_list:
                    index = self.hit_list.index(end.tolist())
                    self.acc_map_list[index][3] = dose + self.acc_map_list[index][3]

                else:
                    self.hit_list.append(end.tolist())
                    self.acc_map_list.append([end.tolist()[0], end.tolist()[1], end.tolist()[2], dose - self.required_dose])

        # create an array that has the accumulation map values
        arr = np.array(self.acc_map_list)
        self.accumulation_map.data = (np.array(arr.ravel(), dtype=np.float32))
        self.accumulation_map.header.stamp = rospy.Time.now()

        # Publish the array
        self.acc_map_pub.publish(self.accumulation_map)

        # Set new prev_time as current time before the beginning of next loop iteration
        self.prev_time = rospy.get_time()


if __name__=="__main__":
    # Initialize accumulation_map node
    rospy.init_node('accumulation_map',anonymous=True)

    # Instantiate the AccumulationMap class
    AccumulationMap()
    rospy.spin()
