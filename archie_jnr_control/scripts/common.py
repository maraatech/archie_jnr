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
from cares_msgs.msg import ArchieRailCmdAction, ArchieRailCmdGoal, ArchieRailCmdFeedback, ArchieRailCmdResult

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

def rail_to_str(status):
    if status == ArchieRailCmdFeedback.INIT:
        return "Init"
    elif status == ArchieRailCmdFeedback.MOVING:
        return "Moving"
    elif status == ArchieRailCmdFeedback.RESETTING:
        return "Resetting"
    elif status == ArchieRailCmdFeedback.STARTING:
        return "Starting"
    elif status == ArchieRailCmdFeedback.HOMING:
        return "Homing"
    return f"Unkown: {status}"

def create_navigation_goal(command):
    navigation_goal = NavigationGoal()
    return navigation_goal

def create_mapping_goal(command, init_pose, get_marker, path_id, scanning_goals, metric_goal, world_link="ground"):
    mapping_goal = MappingGoal()
    mapping_goal.command = command
    mapping_goal.init_pose       = init_pose
    mapping_goal.get_marker.data = get_marker
    mapping_goal.path_id         = path_id
    mapping_goal.scanning_goals  = scanning_goals
    mapping_goal.metric_goal     = metric_goal
    mapping_goal.world_link.data = world_link

    return mapping_goal

def create_scanning_goal(command, planning_link=None, file_path=""):
    scanning_goal = ScanningGoal()
    scanning_goal.command            = command
    scanning_goal.planning_link.data = planning_link
    scanning_goal.file_path.data     = file_path
    return scanning_goal

def create_metric_goal(command, file_path=""):
    metric_goal = MetricExtractionGoal()
    metric_goal.command        = command
    metric_goal.file_path.data = file_path
    return metric_goal