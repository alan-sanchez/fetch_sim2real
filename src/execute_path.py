#!/usr/bin/env python

# Import modules
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np

# Import message types and other python libraries.
from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotTrajectory
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class ExecutePath(object):
    """
    Class that commands the Fetch robot to execute a joint trajectory based on
    subscribes pose goals.
    """
    def __init__(self):
        """
        A function that initializes subscriber, publisher, moveit_commander,and
        planning scene.
        :param self: The self reference.
        """
        super(ExecutePath, self).__init__()
        # Initialize subscribers
        self.waypoints_sub  = rospy.Subscriber('waypoints',  PoseArray,         self.callback_waypoints)
        self.velocities_sub = rospy.Subscriber('velocities', numpy_msg(Floats), self.go_to_pose)
        # self.pause_sub      = rospy.Subscriber('')

        # Initialize Publisher
        self.start_pub = rospy.Publisher('start', String, queue_size=10)
        self.stop_pub  = rospy.Publisher('stop',  String, queue_size=10)

        # First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level
        # interface to the robot
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.
        self.group = moveit_commander.MoveGroupCommander("arm_with_torso")
        self.group.set_end_effector_link("ee_link") #use gripper_link if it is planar disinfection

        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        # Initialize waypoints variable
        self.waypoints = None

        # This creates objects in the planning scene that mimic the table
        # If these were not in place gripper could hit the table
        self.planning_scene = PlanningSceneInterface("base_link")
        # self.planning_scene.removeCollisionObject("Table")
        # self.planning_scene.addBox("Table", size_x=1, size_y=1.5, size_z=0.05, x=1, y=0, z=0.625 )

        # Print out instructions for user input to get things started
        print("")
        print("====== Press 'Enter' to generate random region =======")

    def callback_waypoints(self,msg):
        """
        Function that stores the PoseArray messages.
        :param self: The self reference.
        :param msg: The PoseArray message type.
        """
        self.waypoints = msg

    def go_to_pose(self,msg):
        """
        Function that sends pose goals for the Fetch to execute. This function also
        publishes a string that will command other nodes to run UV accumulation
        functions.
        :param self: The self reference.
        :param msg: The PoseArray message type.
        """
        # Publish string command to initiate functions in other nodes
        self.start_pub.publish("start")

        # Go to each pose goal, using a forloop
        for pose_goal, vel in zip(self.waypoints.poses, msg.data):
            self.group.set_pose_target(pose_goal)
            self.group.set_planning_time(5)

            # Set the velocity for Joint trajectory goal
            self.group.set_max_velocity_scaling_factor(vel.item())

            # Execute the pose goal
            plan = self.group.go(wait=True)

        # Publish string command to stop UV accumulation mapping from other nodes
        self.stop_pub.publish("stop")

        # Go back to initail position
        self.init_pose()

        # Pause the simulation for a couple of seconds
        rospy.sleep(2.5)

        # Print out instructions for user input for next disinfection simulation
        print("")
        print("====== Press 'Enter' to generate random region =======")


    def init_pose(self, vel = 0.2):
        """
        Function that sends a joint goal that moves the Fetch's arm and torso to
        the initial position.
        :param self: The self reference.
        :param vel: Float value for arm velocity.
        """
        # Set the velocity for Joint trajectory goal
        self.group.set_max_velocity_scaling_factor(vel)

        # List of joint positions for the initial pose
        joints = [.05, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
        plan = self.group.go(joints, wait=True)
       


if __name__ == '__main__':
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    rospy.init_node('execute_path', anonymous=True)

    # Instantiate the ExecutePath class
    ExecutePath()
    rospy.spin()
