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
    def __init__(self, navigation_server=None, side_controllers=None):
        self.navigation_controller = None if navigation_server == None else NavigationController(navigation_server)
        self.task_controller       = None if side_controllers  == None else TaskModuleController(side_controllers)

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
        return True

    def start_navigation(self, navigation_goal, home_goal):
        # Logic to determine if navigation is able to be started
        if self.navigation_controller == None:
            return False

        # Check if the mapping controller is operating or not
        if not self.task_controller == None and not self.task_controller.is_idle():
            print(f"Mapping is still operating: {common.status_to_str(self.task_controller.get_state())}")
            return False
        elif not self.task_controller == None:
            self.start_task(home_goal)

        # Do your navigation stuff here
        self.navigation_controller.start_navigation(navigation_goal)

    def stop(self):
        if not self.navigation_controller == None:
            self.navigation_controller.stop()

        if not self.task_controller == None:
            self.task_controller.stop()

    def get_state(self):
        return None if self.navigation_controller == None else self.navigation_controller.get_state(), None if self.task_controller == None else self.task_controller.get_state()

def create_ui_layout():
    pass

def main():
    rospy.init_node('arhcie_controller', anonymous=True)

    print("Setting up Arhcie Controllers")
    # replace this with array of "sides"
    side_controllers = rospy.get_param('~side_controllers', None)
    print(side_controllers)
    # add reading in navigation server
    navigation_server = rospy.get_param('~navigation_server', None)
    print(navigation_server)

    # Use these as defaults but read them from the UI for future runs
    world_link    = rospy.get_param('~world_link')
    planning_link = rospy.get_param('~planning_link')
    get_marker    = rospy.get_param('~capture_marker', False)
    path_id       = rospy.get_param('~path_id', 0)

    init_x        = rospy.get_param('~init_x')
    init_y        = rospy.get_param('~init_y')
    init_z        = rospy.get_param('~init_z')
    init_roll     = rospy.get_param('~init_roll', 0)
    init_pitch    = rospy.get_param('~init_pitch', 0)
    init_yaw      = rospy.get_param('~init_yaw', 90)
    init_pose     = utils.create_pose_stamped_msg(init_x, init_y, init_z, planning_link, rpy_deg=[init_roll,init_pitch,init_yaw])
    
    archie_controller = ArchieController(navigation_server=navigation_server, side_controllers=side_controllers)

    # Improve UI
    layout = []
    layout.append([sg.Text('Nav-State:'), sg.Text(size=(45,1), key='-NAV-STATE-')])

    if not side_controllers == None:
        size = (5,1)
        layout.append([sg.Text("Init X"), sg.InputText(init_x, size=size, key='-INIT-X-'), 
            sg.Text("Init Y"), sg.InputText(init_y, size=size, key='-INIT-Y-'), 
            sg.Text("Init Z"), sg.InputText(init_z, size=size, key='-INIT-Z-')])

        layout.append([sg.Text("Init R"), sg.InputText(init_roll, size=size, key='-INIT-R-'), 
            sg.Text("Init P"), sg.InputText(init_pitch, size=size, key='-INIT-P-'), 
            sg.Text("Init Y"), sg.InputText(init_yaw, size=size, key='-INIT-Y-')])

        for i, side_controller in enumerate(side_controllers):
            layout.append([sg.Text(f"Side {i}"), sg.Text(size=(30,1), key=side_controller)])

    layout.append([sg.Button('Map'), sg.Button('Home'), sg.Button('Navigate'), sg.Button('Stop'), sg.Button('Exit')])

    window = sg.Window('Archie Control Interface', layout)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #Process any UI inputs
        event, values = window.read(timeout=100)

        if event == sg.WIN_CLOSED or event == 'Exit':
            print("Exiting")
            break

        if event == 'Map':
            task_goal = common.create_task_goal(MappingGoal.MAP, init_pose, planning_link=planning_link, world_link=world_link, get_marker=get_marker, path_id=path_id)
            if archie_controller.start_task(task_goal):
                print("Mapping Task Started")
            else:
                print("Mapping Request Failed")

        elif event == 'Navigate':
            navigation_goal = common.create_navigation_goal(0)
            task_goal       = common.create_task_goal(MappingGoal.MOVE, init_pose)
            if archie_controller.start_navigation(navigation_goal, home_goal=task_goal):
                print("Navigation Task Started")
            else:
                print("Navigation Request Failed")            

        elif event == 'Home':
            task_goal = common.create_task_goal(MappingGoal.MOVE, init_pose)
            if archie_controller.start_task(task_goal):
                print("Home Task Started")
            else:
                print("Home Request Failed")   

        elif event == 'Stop':
            archie_controller.stop()
            print("Stop Sent")
        
        #Update UI with state
        navigation_status, task_state = archie_controller.get_state()
        navigation_status = common.status_to_str(navigation_status)
        window['-NAV-STATE-'].update(navigation_status)

        for side_controller, state, task_status in task_state:
            status = common.status_to_str(state)+" "+task_status
            window[side_controller].update(status)

        #Update init_pose from UI
        init_pose = utils.create_pose_stamped_msg(float(values["-INIT-X-"]), 
                                                  float(values["-INIT-Y-"]), 
                                                  float(values["-INIT-Z-"]), 
                                                  planning_link, 
                                                  rpy_deg=[float(values["-INIT-R-"]), 
                                                           float(values["-INIT-P-"]),
                                                           float(values["-INIT-Y-"])])

        r.sleep()

    window.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
