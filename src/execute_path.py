#!/usr/bin/env python

import sys
import actionlib
import subprocess
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
import time

from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotTrajectory
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray


class ExecutePath(object):
    """
    MoveGroupPythonIntefaceTutorial
    """

    def __init__(self):
        super(ExecutePath, self).__init__()
        # Initialize subscribers
        self.waypoints_sub = rospy.Subscriber('waypoints', PoseArray, self.go_to_pose)

        # Publisher
        self.duration_pub  = rospy.Publisher('duration', Float32, queue_size=10)

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
        # Initialize waypoints
        self.waypoints = None

        # Getting Basic Information
        self.planning_frame = self.group.get_planning_frame()

        # Set path_to_goal to the FollowTrajectoryClient Class
        self.path_to_goal=FollowTrajectoryClient()
        self.path_to_goal.init_pose()
        print("====== Press 'Enter' to generate random region =======")



    def go_to_pose(self,msg):
        # flag = False
        self.duration_pub.publish(.5)

        for pose_goal in msg.poses:
            self.group.set_pose_target(pose_goal)
            self.group.set_planning_time(5)
            # plan = group.plan()
            # self.group.set_max_acceleration_scaling_factor(.5)
            # if not flag:
            #     pass
            # else:
            #     print("")
            #     print("time for stop: " + str(time.time()-stop))


            # t = time.time()
            # Now, we call the planner to compute the plan and execute it.
            plan = self.group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            # elapsed = time.time() - t
            # stop = time.time()
            # flag = True
            #
            # print("time to execute: " + str(elapsed))
            # print("")

        self.path_to_goal.init_pose()
        rospy.sleep(2)
        print("====== Press 'Enter' to generate random region =======")




class FollowTrajectoryClient(object):

    def __init__(self):
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")
        self.scene = PlanningSceneInterface("base_link")
        self.scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

    def init_pose(self, vel = .4):

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")

        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [.05, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     tolerance = 0.02,
                                                     max_velocity_scaling_factor=vel)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                rospy.loginfo("done")
                return



if __name__ == '__main__':
    ## First initialize `moveit_commander`_ and a `rospy`_ node:

    rospy.init_node('execute_path', anonymous=True)
    ExecutePath()
    rospy.spin()
