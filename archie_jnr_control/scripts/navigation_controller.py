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
from cares_msgs.msg import MappingAction, MappingGoal, MappingFeedback, MappingResult

from geometry_msgs.msg import Pose, PoseStamped

from std_msgs.msg import Header

import cares_lib_ros.utils as utils

class NavigationController(object):
    def __init__(self, navigation_server):
        self.navigation_client = actionlib.SimpleActionClient(navigation_server, NavigationAction)
        self.navigation_client.wait_for_server()

        self.idle = True

    def callback_active(self):
        self.idle = False

    def callback_done(self, state, result):
        rospy.loginfo("Navigation server is done. State: %s, result: %s" % (str(state), str(result)))
        self.idle = True

    def callback_feedback(self, feedback):
        self.idle = False

    def send_goal(self, goal):
        self.idle = False
        self.navigation_client.send_goal(goal,
                                      active_cb=self.callback_active,
                                      feedback_cb=self.callback_feedback,
                                      done_cb=self.callback_done)

    def start_navigation(self, navigation_goal):
        self.send_goal(goal)

    def idle(self):
        return self.idle


class ArchieController(object):
    def __init__(self, navigation_server=None, mapping_servers=None):
        self.navigation_controller = None if navigation_server == None else NavigationController(navigation_server)
        self.task_controller       = None if mapping_servers == None else TaskModuleController(mapping_servers)

    def start_task(self, task_goal):
        # Logic to determine if mapping is able to be started

        # Check if a task controller is operating
        if self.task_controller == None:
            print("No Task Controller Operating")
            return False

        # Check if the navigation controller is operating or not
        if not self.navigation_controller == None and not self.navigation_controller.is_idle():
            print(f"Navigation is still operating: {status_to_str(self.navigation_controller.get_state())}")
            return False

        # Start Task suff here
        self.task_controller.start_task(task_goal)

    def start_navigation(self, navigation_goal):
        # Logic to determine if navigation is able to be started
        if self.navigation_controller == None:
            return False

        # Check if the mapping controller is operating or not
        if not self.task_controller == None and not self.task_controller.is_idle():
            print(f"Mapping is still operating: {status_to_str(self.task_controller.get_state())}")
            return False

        # Do your navigation stuff here
        self.navigation_controller.start_navigation(navigation_goal)