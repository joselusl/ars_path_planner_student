#!/usr/bin/env python

import numpy as np
from numpy import *

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader


# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import nav_msgs.msg
from nav_msgs.msg import Path



#
from ars_path_planner_core import *

from ars_path_planner.srv import GetCollisionFreePath, GetCollisionFreePathResponse
from ars_path_planner.srv import CheckPathCollisionFree, CheckPathCollisionFreeResponse


#
import ars_lib_helpers





class ArsPathPlannerRos:

  #######

  # World frame
  world_frame = 'world'

  # Timestamp reference
  time_stamp_reference = rospy.Time(0)

  # Obstacles world subscriber
  obstacles_world_sub = None

  # Service server
  #
  compute_collision_free_path_srv = None
  #
  check_path_collision_free_srv = None

  #
  config_param_yaml_file_name = None

  # Path planner
  path_planner = ArsPathPlanner()
  


  #########

  def __init__(self):

    # Path planner
    self.path_planner = ArsPathPlanner()

    # end
    return


  def init(self, node_name='ars_path_planner_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_path_planner')
    

    #### READING PARAMETERS ###
    
    # Config param
    default_config_param_yaml_file_name = os.path.join(pkg_path,'config','config_path_planner_core.yaml')
    config_param_yaml_file_name_str = rospy.get_param('~config_param_path_planner_core_yaml_file', default_config_param_yaml_file_name)
    print(config_param_yaml_file_name_str)
    self.config_param_yaml_file_name = os.path.abspath(config_param_yaml_file_name_str)

    ###


    # Load config param
    with open(self.config_param_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.config_param = yaml.load(file, Loader=SafeLoader)['path_planner_core']

    if(self.config_param is None):
      print("Error loading config param path planner")
    else:
      print("Config param path planner:")
      print(self.config_param)

    # Config parameters of path planner
    self.path_planner.setConfigParameters(self.config_param)

    # init path planner
    self.path_planner.init()
    
    # End
    return


  def open(self):

    # Subscribers
    #
    self.obstacles_world_sub = rospy.Subscriber('obstacles_world', MarkerArray, self.obstaclesWorldCallback, queue_size=1)
    

    # Service server
    #
    self.compute_collision_free_path_srv = rospy.Service('compute_collision_free_path', GetCollisionFreePath, self.computeCollisionFreePathCallback)
    #
    self.check_path_collision_free_srv = rospy.Service('check_path_collision_free', CheckPathCollisionFree, self.checkPathCollisionFreeCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def setRobotPoseRef(self, robot_pose_ref_msg):

    # Timestamp
    if(robot_pose_ref_msg.header.stamp == rospy.Time(0)):
      self.time_stamp_reference = rospy.Time.now()
    else:
      self.time_stamp_reference = robot_pose_ref_msg.header.stamp

    # Transform message
    # Position
    robot_posi_ref = np.zeros((3,), dtype=float)
    robot_posi_ref[0] = robot_pose_ref_msg.pose.position.x
    robot_posi_ref[1] = robot_pose_ref_msg.pose.position.y
    robot_posi_ref[2] = robot_pose_ref_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat_ref = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat_ref[0] = robot_pose_ref_msg.pose.orientation.w
    robot_atti_quat_ref[1] = robot_pose_ref_msg.pose.orientation.x
    robot_atti_quat_ref[2] = robot_pose_ref_msg.pose.orientation.y
    robot_atti_quat_ref[3] = robot_pose_ref_msg.pose.orientation.z

    robot_atti_quat_simp_ref = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat_ref)

    # Set reference in the path planner
    self.path_planner.setRobotPoseRef(robot_posi_ref, robot_atti_quat_simp_ref)


    #
    return


  def setRobotPose(self, robot_pose_msg):

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_pose_msg.pose.position.x
    robot_posi[1] = robot_pose_msg.pose.position.y
    robot_posi[2] = robot_pose_msg.pose.position.z

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_pose_msg.pose.orientation.w
    robot_atti_quat[1] = robot_pose_msg.pose.orientation.x
    robot_atti_quat[2] = robot_pose_msg.pose.orientation.y
    robot_atti_quat[3] = robot_pose_msg.pose.orientation.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.path_planner.setRobotPose(robot_posi, robot_atti_quat_simp)

    #
    return


  def obstaclesWorldCallback(self, obstacles_world_msg):

    obstacles_world_msg_filtered = MarkerArray()

    for obst_i_msg in obstacles_world_msg.markers:
      if(obst_i_msg.action == 0):
        obstacles_world_msg_filtered.markers.append(obst_i_msg)

    # Set
    self.path_planner.setObstaclesWorld(obstacles_world_msg_filtered)

    #
    return


  def computeCollisionFreePathCallback(self, request):

    #
    robot_pose_msg = request.pose_robot
    self.setRobotPose(robot_pose_msg)

    #
    robot_pose_ref_msg = request.pose_desired
    self.setRobotPoseRef(robot_pose_ref_msg)


    # Call the path planner
    self.path_planner.planPathCall()


    #
    robot_traj_ref_msg = self.robotTrajMsgGen(self.path_planner.robot_traj)
    robot_traj_ref_raw_msg = self.robotTrajMsgGen(self.path_planner.robot_traj_raw)

    #
    serv_response = GetCollisionFreePathResponse()
    serv_response.sucess.data = self.path_planner.flag_set_robot_traj
    serv_response.collision_free_traj = robot_traj_ref_msg
    serv_response.collision_free_traj_raw = robot_traj_ref_raw_msg

    # end
    return serv_response


  def checkPathCollisionFreeCallback(self, request):

    #
    robot_traj_ref_msg = request.trajectory

    # Set
    robot_traj = []
    for pose_i_msg in robot_traj_ref_msg.poses:

      waypoint_i = ars_lib_helpers.PoseSimp()

      waypoint_i.position[0] = pose_i_msg.pose.position.x
      waypoint_i.position[1] = pose_i_msg.pose.position.y
      waypoint_i.position[2] = pose_i_msg.pose.position.z

      attitude_quat = ars_lib_helpers.Quaternion.zerosQuat()
      attitude_quat[0] = pose_i_msg.pose.orientation.w
      attitude_quat[1] = pose_i_msg.pose.orientation.x
      attitude_quat[2] = pose_i_msg.pose.orientation.y
      attitude_quat[3] = pose_i_msg.pose.orientation.z
      waypoint_i.attitude_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(attitude_quat)

      robot_traj.append(waypoint_i)

    # Check
    is_collision_free = self.path_planner.checkPathCollisionFree(robot_traj)

    # Response
    serv_response = CheckPathCollisionFreeResponse()
    serv_response.sucess.data = True
    serv_response.is_collision_free.data = is_collision_free

    # End
    return serv_response


  def robotTrajMsgGen(self, robot_traj):

    # Generate message
    robot_traj_ref_msg = Path()

    robot_traj_ref_msg.header.stamp = self.time_stamp_reference
    robot_traj_ref_msg.header.frame_id = self.world_frame

    robot_traj_ref_msg.poses = []

    if(self.path_planner.flag_set_robot_traj):
      for pose_i in robot_traj:

        pose_i_msg = PoseStamped()

        pose_i_msg.header.stamp = rospy.Time()
        pose_i_msg.header.frame_id = self.world_frame

        pose_i_msg.pose.position.x = pose_i.position[0]
        pose_i_msg.pose.position.y = pose_i.position[1]
        pose_i_msg.pose.position.z = pose_i.position[2]

        pose_i_msg.pose.orientation.w = pose_i.attitude_quat_simp[0]
        pose_i_msg.pose.orientation.x = 0.0
        pose_i_msg.pose.orientation.y = 0.0
        pose_i_msg.pose.orientation.z = pose_i.attitude_quat_simp[1]

        robot_traj_ref_msg.poses.append(pose_i_msg)

    return robot_traj_ref_msg

