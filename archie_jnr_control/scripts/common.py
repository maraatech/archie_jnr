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
from cares_msgs.msg import NavigationAction, NavigationGoal
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal
from cares_msgs.msg import MappingAction, MappingGoal, MappingFeedback, MappingResult

from geometry_msgs.msg import Pose, PoseStamped

from std_msgs.msg import Header

import cares_lib_ros.utils as utils

def status_to_str(status):
    if status == GoalStatus.ACTIVE:
        return "Active"
    elif status == GoalStatus.PREEMPTED:
        return "Preempted"
    elif status == GoalStatus.SUCCEEDED:
        return "Succeeded"
    elif status == GoalStatus.ABORTED:
        return "Aborted"
    elif status == GoalStatus.REJECTED:
        return "Rejected"
    elif status == GoalStatus.RECALLING:
        return "Recalling"
    elif status == GoalStatus.RECALLED:
        return "Recalled"
    elif status == GoalStatus.LOST:
        return "Lost"
    return f"None: {status}"

def create_navigation_goal(command):
    navigation_goal = NavigationGoal()
    return navigation_goal

def create_task_goal(command, pose, planning_link=None, world_link="world", get_marker=False, path_id=255):
    mapping_goal = MappingGoal()
    mapping_goal.command            = command
    mapping_goal.init_pose          = pose
    mapping_goal.planning_link.data = planning_link if not planning_link == None else pose.header.frame_id
    mapping_goal.world_link.data    = world_link
    mapping_goal.get_marker.data    = get_marker
    mapping_goal.path_id            = path_id
    return mapping_goal