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

import common
from task_controller import TaskModuleController
from navigation_controller import NavigationController

import os
from os.path import expanduser
import time
home = expanduser("~")

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
            print(f"Navigation is still operating: {common.status_to_str(self.navigation_controller.get_state())}")
            return False

        # Start Task suff here
        self.task_controller.start_task(task_goal)

    def start_navigation(self, navigation_goal):
        # Logic to determine if navigation is able to be started
        if self.navigation_controller == None:
            return False

        # Check if the mapping controller is operating or not
        if not self.task_controller == None and not self.task_controller.is_idle():
            print(f"Mapping is still operating: {common.status_to_str(self.task_controller.get_state())}")
            return False

        # Do your navigation stuff here
        self.navigation_controller.start_navigation(navigation_goal)

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
            
        window['-STATE-'].update(common.status_to_str(archie_side_a_controller.get_state()))
        r.sleep()

    window.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
