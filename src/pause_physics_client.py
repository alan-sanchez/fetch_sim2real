#!/usr/bin/env python3

# Import module
import rospy 

# Import the needed message type
from std_srvs.srv import Empty

class PausePhysicsClient:
    """

    """
    def __init__(self):
        # Initialize Publisher
        # self.pause_physics_sub = rospy.Publisher('stop_arm_trajectory', String, )
        i = 0

    def sim(self,command,time):
        # Initialize the node
        # rospy.init_node('pause_physics_client')

        # Wait for the service to become available. 
        rospy.wait_for_service(command)

        # Set up a service proxy with the name and message typ of the service. 
        physics_service_client = rospy.ServiceProxy(command, Empty)

        # 
        try:
            physics_service_client()
        except rospy.ServiceException as e:
            print("service call failed: %s"%e)

        rospy.sleep(time)


if __name__ == "__main__":
    object = PausePhysicsClient()
    object.sim('/gazebo/pause_physics', time=5.0)
    object.sim('/gazebo/unpause_physics', time=1.0)

# #!/usr/bin/env python

# # Import python modules
# import numpy as np
# import random
# import rospy
# import sys

# # Import message types and other python libraries
# from scipy.spatial import ConvexHull
# from std_msgs.msg import Header
# from geometry_msgs.msg import PolygonStamped, Point32
# from visualization_msgs.msg import Marker
# from shapely.geometry.polygon import LinearRing

# class Region:
#     """
#     A class that randomly generates a polygon region for the Fetch to disinfect.
#     """
#     def __init__(self):
#         """
#         Initializes publishers and other message types.
#         :param self: The self reference.
#         """
#         # Initialize Publishers
#         self.region_pub = rospy.Publisher('region', PolygonStamped, queue_size=1)

#         # Setup header
#         self.header = Header()
#         self.header.frame_id = "/base_link"
#         self.header.stamp = rospy.Time.now()

#         # Setup self.region as a PolygonStamped message type
#         self.region = PolygonStamped()
#         self.region.header = self.header

#     def convex_hull(self):
#         """
#         Function that uses that convex hull method to create a disinfection region.
#         :param self: The self reference.

#         :publishes self.region: The PolygonStamped Message
#         """

#         # Create an empty list to store 10 random x and y coordinates
#         coord = []
#         for i in range(10):
#             x = random.uniform(0.6, 0.9)
#             y = random.uniform(0.55, -0.1)
#             coord.append([x,y])

#         # Run convex hull function on the random 2D plane coordinates
#         hull = ConvexHull(coord)

#         # Reset the polygon.points for the self.region data type
#         self.region.polygon.points = []
#         line = []

#         # Store the hull vertices to the polygon.points
#         for e in hull.vertices:
#             self.region.polygon.points.append(Point32(coord[e][0], coord[e][1], 0.63))
#             line.append([coord[e][0], coord[e][1]])

#         # Publish polygon regions
#         self.region_pub.publish(self.region)


# if __name__ == '__main__':
#     # Initialize the node
#     rospy.init_node('region_generator')

#     # Declare object from Region class
#     region = Region()

#     # Set rospy rate
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         # Wait for user input before generating new disinfection region
#         raw_input()

#         # Run the convex hull function
#         region.convex_hull()
#         rate.sleep()
