#!/usr/bin/env python3

import numpy as np
from numpy import *

import os


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import nav_msgs.msg
from nav_msgs.msg import Path

#
from ars_path_planner_srvs.srv import GetCollisionFreePath
from ars_path_planner_srvs.srv import CheckPathCollisionFree




class ArsPathPlannerRos(Node):

  #######

  # Robot traj publisher
  robot_traj_ref_pub = None
  robot_traj_ref_raw_pub = None
  #
  robot_traj_ref_msg = None
  robot_traj_ref_raw_msg = None

  # Robot pose subscriber
  robot_pose_sub = None
  #
  robot_pose_msg = None

  # Robot pose ref subscriber
  robot_pose_ref_sub = None
  #
  robot_pose_ref_msg = None

  #
  check_path_loop_timer = None
  check_path_loop_freq = None
  


  #########

  def __init__(self, node_name='ars_path_planner_node'):
    # Init ROS
    super().__init__(node_name)

    #
    self.robot_traj_ref_msg = Path()
    self.robot_traj_ref_raw_msg = Path()

    #
    self.check_path_loop_freq = 1.0

    #
    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_path_planner_node'):

    # Package path
    try:
      pkg_path = get_package_share_directory('ars_path_planner_src')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")
    

    #### READING PARAMETERS ###
    
    # TODO JL

    ###


    # End
    return


  def open(self):

    # Subscribers

    #
    self.robot_pose_ref_sub = self.create_subscription(PoseStamped, 'robot_pose_des', self.robotPoseRefCallback, qos_profile=10)

    # 
    self.robot_pose_sub = self.create_subscription(PoseStamped, 'robot_pose', self.robotPoseCallback, qos_profile=10)
    

    # Publishers

    # 
    self.robot_traj_ref_raw_pub = self.create_publisher(Path, 'robot_trajectory_ref_raw', qos_profile=10)
    # 
    self.robot_traj_ref_pub = self.create_publisher(Path, 'robot_trajectory_ref', qos_profile=10)


    # Services clients
    # Wait for the service to become available
    self.cli_get_collision_free_path = self.create_client(GetCollisionFreePath, 'compute_collision_free_path')
    while not self.cli_get_collision_free_path.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service compute_collision_free_path not available, waiting again...')
    self.get_logger().info('service compute_collision_free_path already available!')

    self.cli_check_path_collision_free = self.create_client(CheckPathCollisionFree, 'check_path_collision_free')
    while not self.cli_check_path_collision_free.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service check_path_collision_free not available, waiting again...')
    self.get_logger().info('service check_path_collision_free already available!')



    # Timers

    # Collision free check
    self.check_path_loop_timer = self.create_timer(1.0/self.check_path_loop_freq, self.checkPathLoopTimerCallback)

    
    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def robotPoseRefCallback(self, robot_pose_ref_msg):

    # Check
    if(self.robot_pose_msg is None):
      self.get_logger().info("Robot pose not received")
      return

    # Check if message repeated
    if(self.robot_pose_ref_msg is not None):
      if(self.robot_pose_ref_msg.pose == robot_pose_ref_msg.pose):
        return

    #
    self.robot_pose_ref_msg = robot_pose_ref_msg

    #
    self.computeCollisionFreeTraj()    

    #
    return


  def robotPoseCallback(self, robot_pose_msg):

    self.robot_pose_msg = robot_pose_msg

    #
    return


  def checkPathLoopTimerCallback(self):

    # Pre-checks
    if(self.robot_traj_ref_msg is None):
      return

    if(not self.robot_traj_ref_msg.poses):
      return

    # Request
    request = CheckPathCollisionFree.Request()
    request.trajectory = self.robot_traj_ref_msg

    # Call service
    try:
      future = self.cli_check_path_collision_free.call_async(request)
      future.add_done_callback(self.checkPathCollisionFreeCallback)
    except Exception as e:
      self.get_logger().info("Service call failed: %s"%e)
      return

    #
    return


  def checkPathCollisionFreeCallback(self, future):

    #
    serv_response = future.result()

    # service response
    success = serv_response.success.data
    is_collision_free = serv_response.is_collision_free.data

    #
    if(not success):
      self.get_logger().info("Error in checking if the trajectory is collision free")
      return   

    #
    if(is_collision_free):
      return
    
    # If not collision free

    # Reset trajectory
    self.robot_traj_ref_msg = Path()
    self.robot_traj_ref_raw_msg = Path()
    #
    self.robotTrajRefPublish()

    # Compute new collision-free trajectory
    self.computeCollisionFreeTraj()

    #
    return


  def computeCollisionFreeTraj(self):

    # Service request
    request = GetCollisionFreePath.Request()
    request.pose_robot = self.robot_pose_msg
    request.pose_desired = self.robot_pose_ref_msg

    # Call service
    try:
      future = self.cli_get_collision_free_path.call_async(request)
      future.add_done_callback(self.getCollisionFreePathCallback)
    except Exception as e:
      self.get_logger().info("Service call failed: %s"%e)
      return

    #
    return


  def getCollisionFreePathCallback(self, future):

    serv_response = future.result()

    # Service response
    success_traj_planning = serv_response.success.data
    self.robot_traj_ref_msg = serv_response.collision_free_traj
    self.robot_traj_ref_raw_msg = serv_response.collision_free_traj_raw

    # Publish planned path
    if(success_traj_planning):
      self.robotTrajRefPublish()
    else:
      self.get_logger().info("Trajectory planner didnt succeed!")

    return


  def robotTrajRefPublish(self):

    # Publish
    self.robot_traj_ref_pub.publish(self.robot_traj_ref_msg)
    self.robot_traj_ref_raw_pub.publish(self.robot_traj_ref_raw_msg)

    #
    return

