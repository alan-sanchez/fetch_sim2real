#!/usr/bin/env python3

# Import modules
import rospy
import numpy as np
import octomap

# Import message types and other pyton libraries
from scipy import spatial
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
        # Initialize Subscribers
        self.vector_sub      = rospy.Subscriber('/vectors',        numpy_msg(Floats), self.irradiance_vectors)
        self.pointcloud_sub  = rospy.Subscriber('/filtered_cloud', PointCloud,        self.callback_pointcloud)
        self.stop_sub        = rospy.Subscriber('/stop',           String,            self.callback_stop_command)

        # Initialize Publishers
        self.accumulation_map_pub = rospy.Publisher('/accumulation_map', HeaderArray, queue_size=10)
        self.proximity_sensor_pub = rospy.Publisher('/proximity_sensor', HeaderArray, queue_size=10)

        # Create an empty list for future storage of the rays that hit
        self.hit_list = []

        # Assign self.accumulation_map as a HeaderArray() message type
        self.accumulation_map = HeaderArray()
        self.accumulation_map.header = Header()
        self.accumulation_map.header.frame_id = "/base_link"

        # Assign self.proximity_sensor as a HeaderArray() message type
        self.proximity_sensor = HeaderArray()
        self.proximity_sensor.header = Header()
        self.proximity_sensor.header.frame_id = "/base_link"

        # Create a list that will store the current accumulation map
        self.temp_acc_map = []

        # The required UV Dose for a UV rate constant of 0.0867 m^2/J is 132.8 (J/m^2)
        self.required_dose = -132.8

        # Set a previous time 
        self.prev_time = rospy.get_time()

        # Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.05
        self.octree = octomap.OcTree(self.resolution)

        # Initialize self.pointcloud variable
        self.pointcloud = None

        # Initialize self.temp_acc_map variable
        self.temp_acc_map = None

        # Initialize command
        self.command = None

    def callback_pointcloud(self, msg):
        """
        Function that stores the filtered point cloud and create new octree for
        castRay calculations.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        # Parse the filtered cloud's points as a np.array. This action is required
        # to pass as an agrument in the insertPointCloud() function.
        self.pointcloud = np.empty(shape=[len(msg.points),3])
        self.temp_acc_map = np.empty(shape=[len(msg.points),4])
        for i in range(len(msg.points)):
            self.pointcloud[i] = [msg.points[i].x,
                                  msg.points[i].y,
                                  msg.points[i].z]

            self.temp_acc_map[i] = [msg.points[i].x,
                                    msg.points[i].y,
                                    msg.points[i].z,
                                    self.required_dose]

        # Insert a 3D scaninto the the tree
        self.octree.insertPointCloud(pointcloud = self.pointcloud, origin = np.array([0, 0, 0], dtype=float))\

        # Instanstiate a `spatial.KDTree()` object and provide self.pointcloud as an arugment
        self.tree = spatial.KDTree(self.pointcloud)

    def callback_stop_command(self, msg):
        """
        A callback function that clears out the octree.
        :param self: The self reference.
        :param msg: The String message type.
        """
        # Deletes the complete tree structure
        self.octree.clear()

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

        # A conditional statement for the first or reset of the self.prev_time variable.
        # If true, then the self.prev_time is defined and the self.hit_list and 
        # self.accumulation_map lists are cleared out
        if self.prev_time == None:
            self.prev_time = rospy.get_time()

        # Compute the UV time exposure 
        time_exposure = abs(rospy.get_time() - self.prev_time)

        # Extract the end effector location data. 
        # NOTE: These coordinates are referencing the base_link transform frame
        ee_loc = np.array([msg.data[0], msg.data[1], msg.data[2]], dtype=np.double)

        # Extract the direction vectors and their irradiance values 
        dir_ir_vectors = msg.data[3:]

        # end is initializes and set as a zero array where the location of the hit will be stored
        end = np.array([0,0,0], dtype=np.double)

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

                # Use the `query()` method to find the closest neigbor. The method
                # returns the distance to the nearest neighbor and theindex
                # of the neighbor in ``self.pointcloud``. 
                neighbor_dist, neighbor_index = self.tree.query(end)

                self.temp_acc_map[neighbor_index][3] = dose + self.temp_acc_map[neighbor_index][3]

                if j == 0:
                    prox_array = np.array(Ray_length)
                    self.proximity_sensor.data = (np.array(prox_array.ravel(), dtype=np.float32))

        # create an array that has the accumulation map values. Then
        # insert the current time for the stamp
        arr = np.array(self.temp_acc_map)
        self.accumulation_map.data = (np.array(arr.ravel(), dtype=np.float32))
        self.accumulation_map.header.stamp = rospy.Time.now()

        # set the time stamp as the same as the accumulation_map header
        self.proximity_sensor.header.stamp = self.accumulation_map.header.stamp

        # Publish the accumulation_map and proximity_sensor messages
        self.accumulation_map_pub.publish(self.accumulation_map)
        self.proximity_sensor_pub.publish(self.proximity_sensor)

        # Set new prev_time as current time before the beginning of next loop iteration
        self.prev_time = rospy.get_time()

if __name__=="__main__":
    # Initialize accumulation_map node
    rospy.init_node('accumulation_map',anonymous=True)

    # Instantiate the AccumulationMap class
    AccumulationMap()
    rospy.spin()
