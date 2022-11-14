#!/usr/bin/env python3

# Import module
import rospy 

# Import the needed message type
from std_srvs.srv import Empty

def sim(command,time):
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
    sim('/gazebo/pause_physics', time=5.0)
    sim('/gazebo/unpause_physics', time=1.0)

