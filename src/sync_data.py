#!/usr/bin/env python3

import rospy
import sys
import message_filters
import pandas as pd
import os

from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, Float32
from sensor_msgs.msg import PointCloud
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from fetch_sim2real.msg import HeaderArray



class SyncData:
    """
    A class that stores messages to its own .csv file.
    """
    def __init__(self):
        """
        A Function that initializes the subscribers, variables, and working directory.
        :param self: The self reference.
        """
        # Initialize subscribers
        self.waypoints_sub  = rospy.Subscriber('/waypoints',      PoseArray,         self.callback_waypoints)
        self.pointcloud_sub = rospy.Subscriber('/filtered_cloud', PointCloud,        self.callback_depth_map)
        self.velocities_sub = rospy.Subscriber('/velocities',     numpy_msg(Floats), self.callback_velocities)
        self.stop_sub       = rospy.Subscriber('/stop',           String,            self.export_data)
        self.stop_sub       = rospy.Subscriber('/start',          String,            self.simulation_number)



        self.accumulation_map_sub = message_filters.Subscriber('/accumulation_map', HeaderArray)
        self.joint_states_sub     = message_filters.Subscriber('/joint_states',     JointState)
        self.ee_pose_sub          = message_filters.Subscriber('/ee_pose',          PoseStamped)

        sync = message_filters.ApproximateTimeSynchronizer([self.accumulation_map_sub,
                                                            self.joint_states_sub,
                                                            self.ee_pose_sub],
                                                            queue_size=10,
                                                            slop=0.1)
        sync.registerCallback(self.callback_sync)

        # Initialize depth_map list
        self.depth_map = []

        # Initialize waypoints list
        self.waypoints = []

        # Initialize velocities List
        self.velocities = []

        # Initiailze command String
        self.command = "start"

        # Get the current working directory of they sync_data node
        cwd =  os.path.dirname(__file__)

        # Change the current working directory to the simulation_data folder
        os.chdir(cwd)
        os.chdir('..')
        os.chdir('simulation_data')

        self.cwd = os.getcwd()

        # Initialize directory path. This will be updated every simulation run
        self.directory = self.cwd
        self.sim_iteration = 0

        # Create boolean for conditional statement for adding headers to .csv files
        self.add_df_header = True


    def callback_depth_map(self, msg):
        """
        Function that stores the filtered point cloud and create new octree for
        castRay calculations.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        self.depth_map = msg

    def callback_waypoints(self,msg):
        """
        Function that stores the PoseArray messages.
        :param self: The self reference.
        :param msg: The PoseArray message type.
        """
        self.waypoints = msg



    def callback_velocities(self,msg):
        """
        Function that stores the Float messages.
        :param self: The self reference.
        :param msg: The Float array message.
        """
        self.velocities = msg


    def export_data(self,msg):
        """
        Function that exports velocity, waypoints, and filtered pointcloud (depth map).
        :param self: The self reference.
        :param msg: The String message.
        """
        # Create dataframe for the waypoint velocities and save to .csv file
        df_vel = pd.DataFrame(self.velocities.data)
        df_vel.to_csv('velocities.csv', index=False, header=False)

        # Create dataframe for waypoints and save to .csv file
        points = []
        for i in range(len(self.waypoints.poses)):
            points.append([self.waypoints.poses[i].position.x,
                        self.waypoints.poses[i].position.y,
                        self.waypoints.poses[i].position.z,
                        self.waypoints.poses[i].orientation.x,
                        self.waypoints.poses[i].orientation.y,
                        self.waypoints.poses[i].orientation.z,
                        self.waypoints.poses[i].orientation.w])
        df_waypoints = pd.DataFrame(points)
        df_waypoints.to_csv('waypoints.csv', index=False, header=False)

        # Create dataframe for depth_map and save to .csv file
        depth_map = []
        for i in range(len(self.depth_map.points)):
            depth_map.append([self.depth_map.points[i].x,
                              self.depth_map.points[i].y,
                              self.depth_map.points[i].z])
        df_depth_map = pd.DataFrame(depth_map)
        df_depth_map.to_csv('depth_map.csv', index=False, header=False)



    def simulation_number(self,msg):
        """
        Function that creates new folder directory for new simulation data.
        :param self: The self reference.
        :param msg: A string message.
        """
        # Iterate before simulation and create updated directory
        self.sim_iteration += 1
        folder = 'simulation_' + str(self.sim_iteration)

        # If previous folder already exists, notify user
        if os.path.exists(folder):
            rospy.logwarn(str(folder) + " already exists!")
        else:
            self.directory = os.path.join(self.cwd, folder)
           # Create new folder directory
            os.mkdir(self.directory)
            os.chdir(self.directory)


    def callback_sync(self, msg1, msg2, msg3):
        """
        Function that sync three different message types, creates a dataframe
        for each othe message, and exports the dataframes to their own .csv file.
        :param self: The self reference.
        :param msg1: A HeaderArray message type.
        :param msg2: A JointState message type.
        :param msg3: A PoseStamp message type.
        """

        # Create dataframe for accumulation_map (df_acc)
        acc_map_data = {}
        acc_map_data['ROS Time'] = msg1.header.stamp.to_sec()
        for i in range(int(len(msg1.data)/4)):
            hit_loc = 'hit_loc_' + str(i)
            acc_map_data[hit_loc] = [msg1.data[i*4 + 0],msg1.data[i*4 + 1],msg1.data[i*4 + 2], msg1.data[i*4 + 3]]

        df_acc = pd.DataFrame([acc_map_data])


        # Create data frame for the joint states (df_joints)
        joint_states_data = {}
        joint_states_data['ROS Time'] = msg2.header.stamp.to_sec()
        for i in range(len(msg2.name)):
            joint_states_data[msg2.name[i]] = msg2.position[i]

        df_joints = pd.DataFrame([joint_states_data])


        # Create dataframe for end effector location (df_ee)
        ee_data = {'ROS Time ': msg3.header.stamp.to_sec(),
                    'Position_x ': msg3.pose.position.x,
                    'Position_y ': msg3.pose.position.y,
                    'Position_z ': msg3.pose.position.z,
                    'Quaternion_x ': msg3.pose.orientation.x,
                    'Quaternion_y ': msg3.pose.orientation.y,
                    'Quaternion_z ': msg3.pose.orientation.z,
                    'Quaternion_w ': msg3.pose.orientation.w}

        df_ee = pd.DataFrame([ee_data])


        # Create and append dataframes to .csv file
        if self.add_df_header:
            df_acc.to_csv('accumulation_map.csv', mode='a', index=False, header=True)
            df_ee.to_csv('ee_location.csv',       mode='a', index=False, header=True)
            df_joints.to_csv('joint_states.csv',  mode='a', index=False, header=True)
            self.add_df_header = False

        else:
            df_acc.to_csv('accumulation_map.csv', mode='a', index=False, header=False)
            df_ee.to_csv('ee_location.csv',       mode='a', index=False, header=False)
            df_joints.to_csv('joint_states.csv',  mode='a', index=False, header=False)


if __name__ == '__main__':
    # Initialize sync_data node
    rospy.init_node('sync_data', argv=sys.argv)

    # Instantiate the SyncData class
    SyncData()
    rospy.spin()
