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
from cares_msgs.msg import ScanningAction, ScanningGoal, ScanningFeedback, ScanningResult
from cares_msgs.msg import MetricExtractionAction, MetricExtractionGoal, MetricExtractionFeedback, MetricExtractionResult

from geometry_msgs.msg import Pose, PoseStamped

from std_msgs.msg import Header

import cares_lib_ros.utils as utils

import common
from module_controller import ModuleController
from navigation_controller import NavigationController

import os
from os.path import expanduser
import time
home = expanduser("~")

def get_filepath():
    now = datetime.now()
    now = now.strftime("%Y-%m-%d-%H-%M-%S")
    scans_directory = "~/canopy_scans/" # Note the "~" gets expanded on the host machine

    number_of_scans = len(list(os.listdir(expanduser(scans_directory))))
    filepath = scans_directory+str(number_of_scans+1)+"_"+now+"/"

    return filepath

class ArchieController(object):
    def __init__(self, navigation_server=None, arm_servers=None, mapping_server=None, rail_servers=None):
        self.navigation_controller = None if navigation_server == None else NavigationController(navigation_server)
        self.module_controller     = None if mapping_server    == None else ModuleController(arm_servers, mapping_server, rail_servers)

    def navigation_active(self):
        if self.navigation_controller is not None and not self.navigation_controller.is_idle():
            return True
        return False

    def module_active(self):
        if self.module_controller is not None and not self.module_controller.is_idle():
            return True
        return False

    def home(self, home_poses, control_links, rail_home):
        # Check if a task controller is operating
        if self.module_controller == None:
            print("No Task Controller Operating")
            return False

        # Check if the navigation controller is operating or not
        if self.navigation_active():
            print(f"Navigation is still operating")
            return False

        self.module_controller.home(home_poses, control_links, rail_home)
        return True
    
    def start_mapping(self, mapping_goal):
        # Logic to determine if mapping is able to be started

        # Check if a task controller is operating
        if self.module_controller == None:
            print("No Task Controller Operating")
            return False

        # Check if the navigation controller is operating or not
        if self.navigation_active():
            print(f"Navigation is still operating")
            return False

        # Start Module suff here
        self.module_controller.start_mapping(mapping_goal)
        return True

    def start_navigation(self, navigation_goal, home_poses, control_links):
        # Logic to determine if navigation is able to be started
        if self.navigation_controller == None:
            return False

        # Check if the mapping controller is operating or not
        if self.module_active():
            print(f"Module is still operating")
            return False
        elif not self.module_controller == None:
            self.home(home_poses, control_links)

        # Do your navigation stuff here
        self.navigation_controller.start_navigation(navigation_goal)
        return True

    def stop(self):
        if not self.navigation_controller == None:
            self.navigation_controller.stop()

        if not self.module_controller == None:
            self.module_controller.stop()

    def get_state(self):
        navigation_state = None if self.navigation_controller == None else self.navigation_controller.get_state()
        module_state     = None if self.module_controller == None else self.module_controller.get_state()
        return navigation_state, module_state 

def rail_ui(key):
    return [[sg.Text(f"Rail:"), sg.Text(size=(40,1), key=key)]]

def arm_ui(key):
    return [[sg.Text(f"Arm:"), sg.Text(size=(40,1), key=key)]]

def scanning_ui(key):
    return [[sg.Text(f"Scan:"), sg.Text(size=(40,1), key=key)]]

def metric_ui(key):
    return [[sg.Text(f"Metric:"), sg.Text(size=(40,1), key=key)]]

def create_layout(navigation_ui, mapping_ui, arm_servers, rail_servers):
    # Use these as defaults but read them from the UI for future runs
    world_link          = rospy.get_param('~world_link')
    
    planning_links       = rospy.get_param('~planning_links')
    robot_control_frames = rospy.get_param('~robot_control_frames')

    get_marker          = rospy.get_param('~capture_marker', False)
    path_id             = rospy.get_param('~path_id', 0)

    init_x        = rospy.get_param('~init_x', 0)
    init_y        = rospy.get_param('~init_y', 0)
    init_z        = rospy.get_param('~init_z', 0)
    init_roll     = rospy.get_param('~init_roll', 0)
    init_pitch    = rospy.get_param('~init_pitch', 0)
    init_yaw      = rospy.get_param('~init_yaw', 90)

    rail_x        = rospy.get_param('~rail_x', 0)
    rail_z        = rospy.get_param('~rail_z', 0)

    # Improve UI
    layout = []    
    if navigation_ui:
        layout.append([sg.Text(f"Navigation Interface")])
        layout.append([sg.Text('Nav-State:'), sg.Text(size=(45,1), key='-NAV-STATE-')])

    if mapping_ui:
        size = (5,1)
        layout.append([sg.Text(f"Arm Interface")])
        layout.append([sg.Text(f"Home Pose")])
        layout.append([sg.Text("Init X"), sg.InputText(init_x, size=size, key='-INIT-X-'), 
            sg.Text("Init Y"), sg.InputText(init_y, size=size, key='-INIT-Y-'), 
            sg.Text("Init Z"), sg.InputText(init_z, size=size, key='-INIT-Z-')])

        layout.append([sg.Text("Init R"), sg.InputText(init_roll, size=size, key='-INIT-ROLL-'), 
            sg.Text("Init P"), sg.InputText(init_pitch, size=size, key='-INIT-PITCH-'), 
            sg.Text("Init Y"), sg.InputText(init_yaw, size=size, key='-INIT-YAW-')])

        size = (20,1)
        layout.append([sg.Text(f"World: "),    sg.InputText(world_link, size=size, key="-WORLD-")])
    
        if rail_servers is not None:
            layout.append([sg.Text(f"Rail Pose")])
            size = (5,1)
            layout.append([sg.Text("Rail X"), sg.InputText(rail_x, size=size, key='-RAIL-X-'), 
            sg.Text("Rail Z"), sg.InputText(rail_z, size=size, key='-RAIL-Z-')])

            for i, arm_server in enumerate(rail_servers):            
                layout.append(rail_ui(arm_server))

        if arm_server is not None:
            layout.append([sg.Text(f"Task Information")])
            size = (20,1)
            for i, arm_server in enumerate(arm_servers):
                layout.append([sg.Text(f"Planning: "), sg.InputText(planning_links[i],       size=size, key=f"-PLANNING-{i}-")])
                layout.append([sg.Text(f"Control: "),  sg.InputText(robot_control_frames[i], size=size, key=f"-CONTROL-{i}-")])
                layout.append(arm_ui(arm_server))

            size = (20,1)
            layout.append([sg.Text(f"Task Interface")])
            layout.append([sg.Text(f"Marker: "), sg.InputText(get_marker, size=size, key="-MARKER-")])
            layout.append([sg.Text(f"Path: "),   sg.InputText(path_id,    size=size, key="-PATH-")])

            layout.append([sg.Text(f"Task: "), sg.Text(size=(30,1), key="-TASK-STATE-")])
            layout.append([sg.Column([[]], key='-Column-')])

    layout.append([sg.Button('Map'), sg.Button('Home'), sg.Button('Navigate'), sg.Button('Stop'), sg.Button('Exit')])
    return layout

def main():
    rospy.init_node('arhcie_controller', anonymous=True)

    print("Setting up Task Controllers")

    rail_servers = rospy.get_param('~rail_servers', None)
    print(f"Rail Servers: {rail_servers}")

    arm_servers = rospy.get_param('~platform_servers', None)
    print(f"Arm Servers: {arm_servers}")

    mapping_server = rospy.get_param('~mapping_server', None)
    print(f"Mapping Server: {mapping_server}")

    navigation_server = rospy.get_param('~navigation_server', None)
    print(f"Navigation Server: {navigation_server}")

    archie_controller = ArchieController(navigation_server=navigation_server, arm_servers=arm_servers, mapping_server=mapping_server, rail_servers=rail_servers)

    layout = create_layout(navigation_server is not None, mapping_server is not None, arm_servers, rail_servers)
    window = sg.Window('Archie Control Interface', layout)

    r = rospy.Rate(10)
    init_pose = None
    while not rospy.is_shutdown():
        #Process any UI inputs
        event, values = window.read(timeout=100)

        if event == sg.WIN_CLOSED or event == 'Exit':
            print("Exiting")
            break

        if event == 'Map':
            # Generate file_path to save data into
            file_path = get_filepath()
            scanning_goals = []
            for i, arm_server in enumerate(arm_servers):
                scanning_goal = common.create_scanning_goal(ScanningGoal.MAP,
                                                            planning_link=str(values[f"-PLANNING-{i}-"]), 
                                                            file_path=file_path)
                scanning_goals.append(scanning_goal)
            
            metric_goal = common.create_metric_goal(MetricExtractionGoal.RESET,
                                                    file_path=file_path)

            mapping_goal = common.create_mapping_goal(MappingGoal.MAP, 
                                                      init_pose,
                                                      bool(int(values["-MARKER-"])), 
                                                      int(values["-PATH-"]),
                                                      scanning_goals, 
                                                      metric_goal,
                                                      world_link=str(values["-WORLD-"]))

            if archie_controller.start_mapping(mapping_goal):
                print("Mapping Task Started")
            else:
                print("Mapping Request Failed")

        elif event == 'Navigate':
            navigation_goal = common.create_navigation_goal(0)
            if archie_controller.start_navigation(navigation_goal, home_poses=init_poses, control_links=control_links):
                print("Navigation Task Started")
            else:
                print("Navigation Request Failed")            

        elif event == 'Home':
            if archie_controller.home(home_poses=init_poses, control_links=control_links, rail_home=rail_home):
                print("Home Task Started")
            else:
                print("Home Request Failed")

        elif event == 'Stop':
            archie_controller.stop()
            print("Stop Sent")
        
        #Update UI with state
        if "-NAV-STATE-" in window.AllKeysDict:
            navigation_status = common.goal_to_str(navigation_status)
            window['-NAV-STATE-'].update(navigation_status)
        
        rail_home = utils.create_pose_msg(0, 0, 0)
        if "-RAIL-X-" in window.AllKeysDict:
            rail_home = utils.create_pose_msg(float(values["-RAIL-X-"]), 0, float(values["-RAIL-Z-"]))

        if "-TASK-STATE-" in window.AllKeysDict:
            navigation_status, module_status = archie_controller.get_state()

            task_status, arm_states, rail_states = module_status

            for key in rail_states:
                status, feedback = rail_states[key]
                status      = common.goal_to_str(status)
                rail_status = common.rail_to_str(feedback.status)
                window[key].update(f"{status} {rail_status}")

            for key in arm_servers:
                status, feedback = arm_states[key]
                status = common.goal_to_str(status)
                window[key].update(f"{status}")

            mapping_status, mapping_feedback = task_status
            count = mapping_feedback.count
            total = mapping_feedback.total
            
            for i, feedback in enumerate(mapping_feedback.scanning_feedback):
                if not f"-MAP-{i}" in window.AllKeysDict:
                    window.extend_layout(window['-Column-'], scanning_ui(f"-MAP-{i}"))
                
                status     = common.scanning_to_str(feedback.status)
                arm_status = common.scanning_to_str(feedback.arm_status)
                window[f"-MAP-{i}"].update(f"{status} Arm: {arm_status} Count: {count}/{total}")

            for i, feedback in enumerate(mapping_feedback.metric_feedback):
                if not f"-METRIC-{i}" in window.AllKeysDict:
                    window.extend_layout(window['-Column-'], metric_ui(f"-METRIC-{i}"))

                status    = common.metric_to_str(feedback.status)
                processed = feedback.processed
                queued    = feedback.queued
                window[f"-METRIC-{i}"].update(f"{status} {processed}/{queued}")


            task_status    = common.goal_to_str(mapping_status)
            mapping_status = common.mapping_to_str(mapping_feedback.status)
            window["-TASK-STATE-"].update(f"{task_status} {mapping_status}")

            #Update init_pose from UI
            init_poses = {}
            control_links = {}
            for i, arm_server in enumerate(arm_servers):
                init_pose = utils.create_pose_stamped_msg(float(values["-INIT-X-"]), 
                                                        float(values["-INIT-Y-"]), 
                                                        float(values["-INIT-Z-"]), 
                                                        str(values[f"-PLANNING-{i}-"]),
                                                        rpy_deg=[float(values["-INIT-ROLL-"]), 
                                                                float(values["-INIT-PITCH-"]),
                                                                float(values["-INIT-YAW-"])])
                init_poses[arm_server]    = init_pose
                control_links[arm_server] = str(values[f"-CONTROL-{i}-"])

        r.sleep()

    window.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
