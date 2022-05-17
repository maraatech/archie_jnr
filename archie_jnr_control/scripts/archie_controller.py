#!/usr/bin/env python3
from enum import Enum, auto
import rospy
import math
import random
from datetime import datetime
import cv2
import open3d as o3d
import numpy as np
import yaml

import PySimpleGUI as sg

import actionlib
from actionlib_msgs.msg import GoalStatus
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal
from cares_msgs.msg import MappingAction, MappingGoal, MappingFeedback, MappingResult

from geometry_msgs.msg import Pose
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from cares_lib_ros.data_sampler import DataSamplerFactory#StereoDataSampler, DepthDataSampler, ZividDepthDataSampler
import cares_lib_ros.utils as utils
from cares_lib_ros.path_factory import PathFactory

import os
from os.path import expanduser
import time
home = expanduser("~")

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
    return "Unkown"

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

def create_navigation_goal():
    navigation_goal = NavigationGoal()
    return navigation_goal

def create_task_goal(command, init_pose, planning_link, world_link, get_marker, path_id):
    mapping_goal = MappingGoal()
    mapping_goal.command            = command
    mapping_goal.init_pose          = init_pose
    mapping_goal.planning_link.data = planning_link
    mapping_goal.world_link.data    = world_link
    mapping_goal.get_marker.data    = get_marker
    mapping_goal.path_id            = path_id
    return mapping_goal

def main():
    rospy.init_node('arhcie_controller', anonymous=True)

    print("Setting up Arhcie Controllers")
    # replace this with array of "sides"
    mapping_servers = []
    mapping_servers.append(rospy.get_param('~mapping_server_a', "mapping_server_a"))
    mapping_servers.append(rospy.get_param('~mapping_server_b', "mapping_server_b"))
    
    # add reading in navigation server

    # Improve UI
    layout = [[sg.Text('State:'), sg.Text(size=(15,1), key='-STATE-')],
             [sg.Input(key='-IN-')],
             [sg.Button('Map'), sg.Button('Stop'), sg.Button('Exit')]]

    window = sg.Window('Mapping Control', layout)

    # Use these as defaults but read them from the UI for future runs
    # world_link    = rospy.get_param('~world_link')
    # planning_link = rospy.get_param('~planning_link')
    # init_x        = rospy.get_param('~init_x')
    # init_y        = rospy.get_param('~init_y')
    # init_z        = rospy.get_param('~init_z')
    # init_roll     = rospy.get_param('~init_roll', 0)
    # init_pitch    = rospy.get_param('~init_pitch', 0)
    # init_yaw      = rospy.get_param('~init_yaw', 90)
    # init_pose     = utils.create_pose_stamped_msg(init_x, init_y, init_z, planning_link, rpy_deg=[init_roll,init_pitch,init_yaw])
    # get_marker    = rospy.get_param('~capture_marker', False)
    # path_id       = rospy.get_param('~path_id', 0)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        event, values = window.read(timeout=100)
        if event == sg.WIN_CLOSED or event == 'Exit':
            break
        if event == 'Map':
            # Update the "output" text element to be the value of "input" element
            archie_side_a_controller.start_mapping(init_pose, planning_link, world_link, get_marker, path_id)
        elif event == 'Stop':
            archie_side_a_controller.stop_mapping()
            
        window['-STATE-'].update(status_to_str(archie_side_a_controller.get_state()))
        r.sleep()

    window.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
