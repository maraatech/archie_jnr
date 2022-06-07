#!/usr/bin/env python3
from enum import Enum, auto
import rospy
import math
import random
from datetime import datetime
import numpy as np
import yaml

import PySimpleGUI as sg

import actionlib
from actionlib_msgs.msg import GoalStatus
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
from cares_msgs.msg import MappingAction, MappingGoal, MappingFeedback, MappingResult
from cares_msgs.msg import ScanningAction, ScanningGoal, ScanningFeedback, ScanningResult
from cares_msgs.msg import MetricExtractionAction, MetricExtractionGoal, MetricExtractionFeedback, MetricExtractionResult

from geometry_msgs.msg import Pose, PoseStamped

from std_msgs.msg import Header

import cares_lib_ros.utils as utils
from cares_lib_ros.action_client import ActionClient

# Task ModuleController will be extended to carry out more than just the mapping task in the future for Archie?
class ModuleController(object):
    def __init__(self, arm_servers, mapping_server):
        # Task Clients
        self.mapping_client = ActionClient(mapping_server, MappingAction, MappingFeedback)
        
        self.arm_clients = {}
        for arm_server in arm_servers:
            self.arm_clients[arm_server] = ActionClient(arm_server, PlatformGoalAction, PlatformGoalFeedback)

    def stop(self):
        if not self.mapping_client.is_idle():
            self.stop_mapping()
        else:
            self.stop_arms()

    def stop_arms(self):
        platform_goal = PlatformGoalGoal()
        platform_goal.command = PlatformGoalGoal.STOP
        for arm_client in self.arm_clients.values():
            arm_client.send_goal(platform_goal) 

    def stop_mapping(self):
        mapping_goal = MappingGoal()
        mapping_goal.command = MappingGoal.STOP
        self.mapping_client.send_goal(mapping_goal)
        while not self.mapping_client.is_idle():
            pass

    def start_mapping(self, mapping_goal):
        self.mapping_client.send_goal(mapping_goal)

    # Pass through link_id
    def home(self, home_poses, control_links):
        if not self.mapping_client.is_idle():
            self.stop_mapping()
        
        for key in home_poses.keys():
            platform_goal = PlatformGoalGoal()
            platform_goal.command = PlatformGoalGoal.MOVE
            platform_goal.link_id.data = control_links[key]
            platform_goal.target_pose  = home_poses[key]
            self.arm_clients[key].send_goal(platform_goal) 

    # Extend state to return arm states as well
    def get_state(self):
        arm_states = {}
        for key, value in self.arm_clients.items():
            arm_states[key] = value.get_state()
        return self.mapping_client.get_state(), arm_states

    def idle(self):
        return self.mapping_client.is_idle()

        