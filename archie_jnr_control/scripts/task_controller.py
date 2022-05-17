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

class ArchieSideController(object):
    def __init__(self, mapping_server):
        self.mapping_client = actionlib.SimpleActionClient(mapping_server, MappingAction)
        self.mapping_client.wait_for_server()

        self.idle = True

    def callback_active(self):
        self.idle = False

    def callback_done(self, state, result):
        rospy.loginfo("Side server is done. State: %s, result: %s" % (str(state), str(result)))
        self.idle = True

    def callback_feedback(self, feedback):
        self.idle = False

    def send_goal(self, goal):
        self.is_idle = False
        self.mapping_client.send_goal(goal,
                                      active_cb=self.callback_active,
                                      feedback_cb=self.callback_feedback,
                                      done_cb=self.callback_done)

    def home(self):
        pass

    def start_mapping(self, mapping_goal):
        self.send_goal(mapping_goal)

    def stop(self):
        mapping_action = MappingGoal()
        mapping_action.command = MappingGoal.STOP
        self.send_goal(mapping_action)

    def get_state(self):
        return self.mapping_client.get_state()

    def is_idle(self):
        return self.idle

class TaskModuleController(object):
    def __init__(self, side_controllers):
        self.archie_side_controllers = {}
        for side_controller in side_controllers:
            self.archie_side_controllers[mapping_server] = ArchieSideController(mapping_server)

    def home_side(self, side):
        self.archie_side_controllers[side].home()

    def home_sides(self):
        for side in self.archie_side_controllers.keys():
            self.home_side(side)

    def start_side_mapping(self, side, mapping_goal):
        self.archie_side_controllers[side].start_mapping(mapping_goal)

    def start_mapping(self, mapping_goal):
        for side in self.archie_side_controllers.keys():
            self.start_mapping(side, mapping_goal)

    def start_task(self, task_goal):
        pass

    def stop_side(self, side):
        self.archie_side_controllers[side].stop()

    def stop(self):
        for side in self.archie_side_controllers.keys():
            self.stop_side(side)

    def get_side_state(self, side):
        return self.archie_side_controllers[side].get_state()        

    def get_state(self):
        state = []
        for side in self.archie_side_controllers.keys()
            state.append(self.get_side_state(side))
        return side

    def idle(self):
        for _, side_controller in self.archie_side_controllers.items():
            if not side_controller.is_idle():
                return False
        return True