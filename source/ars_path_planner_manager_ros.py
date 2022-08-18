#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import nav_msgs.msg
from nav_msgs.msg import Path



#
from ars_path_planner.srv import GetCollisionFreePath
from ars_path_planner.srv import CheckPathCollisionFree


#
import ars_lib_helpers





class ArsPathPlannerRos:

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

  #
  service_call_collision_free_path = None
  #
  service_call_check_path_collision_free = None


  


  #########

  def __init__(self):

    #
    self.robot_traj_ref_msg = Path()
    self.robot_traj_ref_raw_msg = Path()

    #
    self.check_path_loop_freq = 1.0



    # end
    return


  def init(self, node_name='ars_path_planner_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_path_planner')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###



    # End
    return


  def open(self):

    # Subscribers

    #
    self.robot_pose_ref_sub = rospy.Subscriber('robot_pose_des', PoseStamped, self.robotPoseRefCallback, queue_size=1)

    # 
    self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robotPoseCallback, queue_size=1)
    

    # Publishers

    # 
    self.robot_traj_ref_raw_pub = rospy.Publisher('robot_trajectory_ref_raw', Path, queue_size=1)
    # 
    self.robot_traj_ref_pub = rospy.Publisher('robot_trajectory_ref', Path, queue_size=1)


    # Services clients
    #
    rospy.wait_for_service('compute_collision_free_path')
    try:
        self.service_call_collision_free_path = rospy.ServiceProxy('compute_collision_free_path', GetCollisionFreePath)
    except rospy.ServiceException as e:
        print("Service creation failed: %s"%e)

    #
    rospy.wait_for_service('check_path_collision_free')
    try:
        self.service_call_check_path_collision_free = rospy.ServiceProxy('check_path_collision_free', CheckPathCollisionFree)
    except rospy.ServiceException as e:
        print("Service creation failed: %s"%e)


    # Timers

    # Collision free check
    self.check_path_loop_timer = rospy.Timer(rospy.Duration(1.0/self.check_path_loop_freq), self.checkPathLoopTimerCallback)

    
    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotPoseRefCallback(self, robot_pose_ref_msg):

    # Check
    if(self.robot_pose_msg is None):
      print("Robot pose not received")
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


  def checkPathLoopTimerCallback(self,timer_msg):

    # Pre-checks
    if(self.robot_traj_ref_msg is None):
      return

    if(not self.robot_traj_ref_msg.poses):
      return

    # Call service
    try:
      serv_response = self.service_call_check_path_collision_free(self.robot_traj_ref_msg)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
      return

    # service response
    sucess = serv_response.sucess.data
    is_collision_free = serv_response.is_collision_free.data

    #
    if(not sucess):
      print("Error in checking if the trajectory is collision freee")
      return

    #
    if(is_collision_free):
      return
    
    # If not collision free

    # Reset trajectory
    #
    self.robot_traj_ref_msg = Path()
    self.robot_traj_ref_raw_msg = Path()
    #
    self.robotTrajRefPublish()

    # Compute new collision-free trajectory
    self.computeCollisionFreeTraj()

    #
    return


  def computeCollisionFreeTraj(self):

    # Call service
    try:
      serv_response = self.service_call_collision_free_path(self.robot_pose_msg, self.robot_pose_ref_msg)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
      return

    # Service response
    success_traj_planning = serv_response.sucess.data
    self.robot_traj_ref_msg = serv_response.collision_free_traj
    self.robot_traj_ref_raw_msg = serv_response.collision_free_traj_raw

    # Publish planned path
    if(success_traj_planning):
      self.robotTrajRefPublish()
    else:
      print("Trajectory planner didnt succeed")

    #
    return


  def robotTrajRefPublish(self):

    # Publish
    self.robot_traj_ref_pub.publish(self.robot_traj_ref_msg)
    self.robot_traj_ref_raw_pub.publish(self.robot_traj_ref_raw_msg)

    #
    return

