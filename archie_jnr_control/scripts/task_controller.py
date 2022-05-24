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

# uint8 INIT      = 0
# uint8 MARKER    = 1
# uint8 CAPTURING = 2
# uint8 ACTUATING = 2
# uint8 ERROR     = 3
# uint8 MOVING    = 4
# uint8 IDLE      = 5

def status_to_str(status):
    if status == MappingFeedback.INIT:
        return "Init"
    elif status == MappingFeedback.MARKER:
        return "Marker"
    elif status == MappingFeedback.CAPTURING:
        return "Capturing"
    elif status == MappingFeedback.IDLE:
        return "Idle"
    elif status == MappingFeedback.MOVING:
        return "Moving"
    elif status == MappingFeedback.ERROR:
        return "Error"
    return f"Unkown: {status}"

class ArchieSideController(object):
    def __init__(self, side_controller):
        self.side_controller = side_controller
        self.side_client = actionlib.SimpleActionClient(self.side_controller, MappingAction)
        self.side_client.wait_for_server()

        self.task_status = "Lost"
        self.idle = True

    def callback_active(self):
        self.idle = False

    def callback_done(self, state, result):
        rospy.loginfo("Side server is done. State: %s, result: %s" % (str(state), str(result)))
        self.idle = True
        self.status = "Done"

    def callback_feedback(self, feedback):
        self.idle = False
        self.task_status = f"{status_to_str(feedback.status)} {status_to_str(feedback.arm_status)} {feedback.count} / {feedback.total}" 

    def send_goal(self, goal):
        self.is_idle = False
        self.side_client.send_goal(goal,
                                      active_cb=self.callback_active,
                                      feedback_cb=self.callback_feedback,
                                      done_cb=self.callback_done)

    def start_task(self, task_goal):
        self.send_goal(task_goal)

    def stop(self):
        task_goal = MappingGoal()
        task_goal.command = MappingGoal.STOP
        self.send_goal(task_goal)

    def get_state(self):
        return self.side_controller, self.side_client.get_state(), self.task_status

    def is_idle(self):
        return self.idle

class TaskModuleController(object):
    def __init__(self, side_controllers):
        self.archie_side_controllers = {}
        for side_controller in side_controllers:
            print(f"Setting up controller {side_controller}")
            self.archie_side_controllers[side_controller] = ArchieSideController(side_controller)

    def start_side_task(self, side, task_goal):
        self.archie_side_controllers[side].start_task(task_goal)

    def start_task(self, task_goal):
        for side in self.archie_side_controllers.keys():
            self.start_side_task(side, task_goal)

    def stop_side(self, side):
        self.archie_side_controllers[side].stop()

    def stop(self):
        for side in self.archie_side_controllers.keys():
            self.stop_side(side)

    def get_side_state(self, side):
        return self.archie_side_controllers[side].get_state()

    def get_state(self):
        state = []
        for side in self.archie_side_controllers.keys():
            state.append(self.get_side_state(side))
        return state

    def idle(self):
        for _, side_controller in self.archie_side_controllers.items():
            if not side_controller.is_idle():
                return False
        return True