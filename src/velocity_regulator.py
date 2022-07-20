#!/usr/bin/env python

# Import modules
import rospy
import numpy as np

# Import message types and other python libraries.
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class VelocityRegulator:
    """
    A class that generates a velocity ratio for the subscribed waypoints.
    """
    def __init__(self):
        """
        Initialize subscriber and publisher
        :param self: The self reference.
        """
        # Initialize subscriber
        self.waypoints_sub = rospy.Subscriber('waypoints', PoseArray, self.velocity_regulator)

        # Intialize Publisher
        self.velocities_pub  = rospy.Publisher('velocities', numpy_msg(Floats), queue_size=10)

    def velocity_regulator(self,msg):
        """
        Generate
        :param self: The self reference.
        :param msg: The PoseArray message type.
        """
        # Create an array of ones. The size is the same as the number of waypoints.
        velocities = np.ones(len(msg.poses), dtype=np.float32)

        # Publish the array of velocity ratios.
        self.velocities_pub.publish(velocities)

if __name__ == '__main__':
    # Initialize velocity_regulator node
    rospy.init_node('velocity_regulator', anonymous=True)

    # Instantiate the VelocityRegulator class
    VelocityRegulator()
    rospy.spin()
