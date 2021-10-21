#!/usr/bin/env python

import numpy as np
from numpy import *

import math

import sys

import os


# ROS

import rospy

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
import ars_lib_helpers

from ars_connected_graph import *



###########################
# class ArsDiscreteSearch
###########################
class ArsDiscreteSearch:

  #######

  # Roadmap
  roadmap = None

  # 
  flag_path_found = False
  action_sequence = []
  states_solution = []

  # functions: f(n) = g(n) + h(n)
  computeCostNode = None
  computeCostG = None
  computeCostH = None



  #########

  def __init__(self):

    # Roadmap
    self.roadmap = None

    # 
    self.flag_path_found = False
    self.action_sequence = []
    self.states_solution = []

    # functions: f(n) = g(n) + h(n)
    self.computeCostNode = None
    self.computeCostG = None
    self.computeCostH = None

    # End
    return

  
  def searchInGraph(self, node_init_id, node_goal_id, flag_verbose=False):

    # Clear previous existing search
    self.flag_path_found = False
    self.action_sequence = []
    self.states_solution = []


    ########## TODO BY STUDENT
    # Fill:
    # self.flag_path_found
    # self.action_sequence
    # Use:
    # self.computeCostNode()
    # self.computeCostG()
    # self.computeCostH()
    # self.roadmap.getConnectedNodes()
    


    
    ######## End TODO BY STUDENT
    

    # End
    return


  def planCreation(self, node_init_id, node_goal_id):

    # Clear
    self.states_solution = []

    # Check
    if(not self.flag_path_found):
      return


    ##### TODO BY STUDENT

    # Fill:
    # self.states_solution
    # Use:
    # self.action_sequence
    


    ##### END TODO BY STUDENT


    # End
    return


  def search(self, node_init_id, node_goal_id, flag_verbose=False):

    # Search in graph
    self.searchInGraph(node_init_id, node_goal_id, flag_verbose)

    # Compute the plan
    self.planCreation(node_init_id, node_goal_id)

    # End
    return
