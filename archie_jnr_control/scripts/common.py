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
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal

from cares_msgs.msg import NavigationAction, NavigationGoal

from cares_msgs.msg import MappingAction, MappingGoal, MappingFeedback, MappingResult
from cares_msgs.msg import ScanningAction, ScanningGoal, ScanningFeedback, ScanningResult
from cares_msgs.msg import MetricExtractionAction, MetricExtractionGoal, MetricExtractionFeedback, MetricExtractionResult

from geometry_msgs.msg import Pose, PoseStamped

from std_msgs.msg import Header

import cares_lib_ros.utils as utils

def goal_to_str(status):
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

def mapping_to_str(status):
    if status == MappingFeedback.INIT:
        return "Init"
    elif status == MappingFeedback.SCANNING:
        return "Scanning"
    elif status == MappingFeedback.EXTRACTING:
        return "Extracting"
    elif status == MappingFeedback.ACTUATING:
        return "Actuating"
    elif status == MappingFeedback.STOPPING:
        return "Stopping"
    return f"Unkown: {status}"

def scanning_to_str(status):
    if status == ScanningFeedback.INIT:
        return "Init"
    elif status == ScanningFeedback.MARKER:
        return "Marker"
    elif status == ScanningFeedback.CAPTURING:
        return "Capturing"
    elif status == ScanningFeedback.IDLE:
        return "Idle"
    elif status == ScanningFeedback.MOVING:
        return "Moving"
    elif status == ScanningFeedback.ERROR:
        return "Error"
    return f"Unkown: {status}"

def metric_to_str(status):
    if status == MetricExtractionFeedback.CAPTURING:
        return "Capturing"
    if status == MetricExtractionFeedback.MEASURING:
        return "Measuring"
    return f"Unkown: {status}"

def create_navigation_goal(command):
    navigation_goal = NavigationGoal()
    return navigation_goal

def create_mapping_goal(command, scanning_goal, metric_goal):
    mapping_goal = MappingGoal()
    mapping_goal.command = command
    mapping_goal.scanning_goal = scanning_goal
    mapping_goal.metric_goal   = metric_goal
    return mapping_goal

def create_scanning_goal(command, pose, planning_link=None, world_link="world", get_marker=False, path_id=255, file_path=""):
    scanning_goal = ScanningGoal()
    scanning_goal.command            = command
    scanning_goal.init_pose          = pose
    scanning_goal.planning_link.data = planning_link if not planning_link == None else pose.header.frame_id
    scanning_goal.world_link.data    = world_link
    scanning_goal.get_marker.data    = get_marker
    scanning_goal.path_id            = path_id
    scanning_goal.file_path.data     = file_path
    return scanning_goal

def create_metric_goal(command, file_path=""):
    metric_goal = MetricExtractionGoal()
    metric_goal.command        = command
    metric_goal.file_path.data = file_path
    return metric_goal