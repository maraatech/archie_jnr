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


