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
from cares_msgs.msg import PathPlanningConstraints
from cares_msgs.msg import ArchieRailCmdAction, ArchieRailCmdGoal

from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped

from std_msgs.msg import Header

import cares_lib_ros.utils as utils

import common
from module_controller import ModuleController
from navigation_controller import NavigationController

import os
from os.path import expanduser
import time
home = expanduser("~")

def get_filepath(scan_prefix):
    now = datetime.now()
    now = now.strftime("%Y-%m-%d-%H-%M-%S")
    scans_directory = "~/canopy_scans/" # Note the "~" gets expanded on the host machine

    number_of_scans = len(list(os.listdir(expanduser(scans_directory))))
    filepath = scans_directory+str(scan_prefix)+"_"+now+"/"

    return filepath

class ArchieController(object):
    def __init__(self, servers, task):
        navigation_server = servers["navigation_server"]
        self.navigation_controller = None if navigation_server == None else NavigationController(navigation_server)
        self.module_controller     = ModuleController(servers, task)

    def navigation_active(self):
        if self.navigation_controller is not None and not self.navigation_controller.is_idle():
            return True
        return False

    def module_active(self):
        if self.module_controller is not None and not self.module_controller.is_idle():
            return True
        return False

    def home(self, home_poses, control_links, rail_home, planning_time, fix_end_effector, constraint_type):
        # Check if a task controller is operating
        if self.module_controller == None:
            print("No Task Controller Operating")
            return False

        # Check if the navigation controller is operating or not
        if self.navigation_active():
            print(f"Navigation is still operating")
            return False

        self.module_controller.home(home_poses, control_links, rail_home, planning_time, fix_end_effector, constraint_type)
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
    
    def start_task(self, task_server, task_control_frame, arm_planning_link, task):
        # Check if a task controller is operating
        if self.module_controller == None:
            print("No Task Controller Operating")
            return False

        # Check if the navigation controller is operating or not
        if self.navigation_active():
            print(f"Navigation is still operating")
            return False

        # Start Module suff here
        self.module_controller.start_task(task_server, task_control_frame, arm_planning_link, task)
        return True

    def start_navigation(self, navigation_goal, home_poses, control_links):
        # Logic to determine if navigation is able to be started
        if self.navigation_controller == None:
            return False

        # Check if the mapping controller is opera        for rail_server in rail_servers:ting or not
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

    def modules_idle(self, active_task_server):
        return self.module_controller.idle(active_task_server)

    def get_state(self, active_task_server):
        navigation_state = None if self.navigation_controller == None else self.navigation_controller.get_state()
        module_state     = None if self.module_controller == None else self.module_controller.get_state(active_task_server)
        return navigation_state, module_state 
    
    def start_arms(self, arm_servers):
        for arm_server in arm_servers:
            print(f"")
            prefix = arm_server.split("_")[2]
            print(f"Prefix")
            stop_arms_service_proxy = rospy.ServiceProxy(f"{prefix}/ur_hardware_interface/dashboard/stop", Empty)
            stop_arms_service_proxy(EmptyRequest())
            start_arms_service_proxy = rospy.ServiceProxy(f"{prefix}/ur_hardware_interface/dashboard/play", Empty)
            start_arms_service_proxy(EmptyRequest())

    def stop_arms(self, arm_servers):
        for arm_server in arm_servers:
            stop_arms_service_proxy = rospy.ServiceProxy(arm_server+"dashboard/stop", Empty)
            stop_arms_service_proxy()

    def enable_rails(self, rail_servers):
        rail_server_goal = ArchieRailCmdGoal()
        rail_server_goal.command = ArchieRailCmd.START
        for rail_server in rail_servers:
            pub = rospy.Publisher(rail_server+"goal", ArchieRailCmd, queue_size=10)
            pub.pub(rail_server_goal)

    def stop_rails(self, rail_servers):
        rail_server_goal = ArchieRailCmdGoal()
        rail_server_goal.command = ArchieRailCmd.STOP
        for rail_server in rail_servers:
            pub = rospy.Publisher(rail_server+"goal", ArchieRailCmd, queue_size=10)
            pub.pub(rail_server_goal)

    
def rail_ui(key):
    return [[sg.Text(f"Rail:"), sg.Text(size=(40,1), key=key)]]

def arm_ui(key):
    return [[sg.Text(f"Arm:"), sg.Text(size=(40,1), key=key)]]

def scanning_ui(key):
    return [[sg.Text(f"Scan:"), sg.Text(size=(40,1), key=key)]]

def metric_ui(key):
    return [[sg.Text(f"Metric:"), sg.Text(size=(40,1), key=key)]]

def task_ui(key, actuator):
    return [[sg.Text(f"{actuator}:"), sg.Text(size=(40,1), key=key)]]

def create_layout(servers, actuator):
    navigation_ui = servers["navigation_server"] is not None
    mapping_ui = servers["mapping_server"] is not None
    has_rail_servers = servers["rail_servers"] is not None
    task_ui = servers["task_servers"]
    arm_servers = servers["arm_servers"]
    task_servers = servers["task_servers"] if servers["task_servers"] is not None else []
    # Use these as defaults but read them from the UI for future runs
    world_link          = rospy.get_param('~world_link')
    
    planning_links       = rospy.get_param('~planning_links')
    robot_control_frames = rospy.get_param('~robot_control_frames')
    task_control_frames = rospy.get_param('~task_control_frames', None)

    get_marker          = rospy.get_param('~capture_marker', False)
    path_id             = rospy.get_param('~path_id', 0)

    init_x        = rospy.get_param('~init_x', 0)
    init_y        = rospy.get_param('~init_y', 0)
    init_z        = rospy.get_param('~init_z', 0)
    init_roll     = rospy.get_param('~init_roll', 0)
    init_pitch    = rospy.get_param('~init_pitch', 0)
    init_yaw      = rospy.get_param('~init_yaw', 90)

    if has_rail_servers:
        rail_x        = rospy.get_param('~rail_x', 0)
        rail_z        = rospy.get_param('~rail_z', 0)

    planning_time = rospy.get_param('~allowed_planning_time', 1.0)
    fix_end_effector = rospy.get_param('~fix_end_effector', True)
    constraint_type = rospy.get_param('~default_constraint_type', PathPlanningConstraints.BOX)

    #index corresponding to task server, assume task server and arm servers have same index TODO: Based on name
    active_task_server = task_servers[0] if task_servers else ""

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

        layout.append([sg.Text("Planning Time"), sg.InputText(planning_time, size=size, key='-PLANNING-TIME-')])
        layout.append([sg.Text("Fix End Effector"), sg.InputText(fix_end_effector, size=size, key='-FEE-')])
        layout.append([sg.Text("Constraint Type"), sg.InputText(constraint_type, size=size, key='-CONSTRAINT-')])

        size = (20,1)
        layout.append([sg.Text(f"World: "),    sg.InputText(world_link, size=size, key="-WORLD-")])
    
        if has_rail_servers:
            layout.append([sg.Text(f"Rail Pose")])
            size = (5,1)
            layout.append([sg.Text("Rail X"), sg.InputText(rail_x, size=size, key='-RAIL-X-'), 
            sg.Text("Rail Z"), sg.InputText(rail_z, size=size, key='-RAIL-Z-')])

            layout.append([sg.Text("Rail X MARKER"), sg.InputText(rail_x, size=size, key='-RAIL-X-MARKER-'), 
            sg.Text("Rail Z MARKER"), sg.InputText(rail_z, size=size, key='-RAIL-Z-MARKER-')])

            for i, arm_server in enumerate(servers["rail_servers"]):            
                layout.append(rail_ui(arm_server))

        if arm_servers is not None:
            layout.append([sg.Text(f"Task Information")])
            size = (20,1)
            layout.append([sg.Text("Scan Prefix: "), sg.InputText("", size=size, key='-SCAN-PREFIX-')])
            for i, arm_server in enumerate(arm_servers):
                layout.append([sg.Text(f"Planning: "), sg.InputText(planning_links[i],       size=size, key=f"-PLANNING-{i}-")])
                layout.append([sg.Text(f"Control: "),  sg.InputText(robot_control_frames[i], size=size, key=f"-CONTROL-{i}-")])
                layout.append(arm_ui(arm_server))

            size = (20,1)
            layout.append([sg.Text(f"Task Interface")])

            for i, task_server in enumerate(task_servers):
                layout.append([sg.Text(f"{actuator} Control: "), sg.InputText(task_control_frames[i], size=size, key=f"-TASK-FRAME-{i}-")])
                # layout.append(task_ui(task_server))
            
            if task_servers:
                layout.append([sg.Text(f"Active {actuator} Server: "), sg.InputText(active_task_server, size=size, key="-ACTIVE-TASK-SERVER-")])

            layout.append([sg.Text(f"Marker: "), sg.InputText(get_marker, size=size, key="-MARKER-")])
            layout.append([sg.Text(f"Path: "),   sg.InputText(path_id,    size=size, key="-PATH-")])

            layout.append([sg.Text(f"Task: "), sg.Text(size=(30,1), key="-TASK-STATE-")])

            layout.append([sg.Column([[]], key='-Column-')])
    
    layout.append([sg.Button('Home'), sg.Button('Navigate'), sg.Button('Stop'), sg.Button('Exit')])

    if task_ui:
        layout[-1].insert(2, sg.Button(f'{actuator}'))
    if mapping_ui:
        layout[-1].insert(2, sg.Button('Map'))

    layout.append([sg.Button('Start Arms'), sg.Button('Stop Arms'), sg.Button('Stop Rail')])

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

    actuator = rospy.get_param('~actuator', "Thinning")
    print(f"Acuator: {actuator}")

    task_servers = rospy.get_param('~task_servers', None)
    print(f"Task Servers: {task_servers}")

    servers = {
        "rail_servers": rail_servers,
        "arm_servers": arm_servers,
        "mapping_server": mapping_server,
        "task_servers": task_servers,
        "navigation_server": navigation_server
    }

    archie_controller = ArchieController(servers, actuator)

    layout = create_layout(servers, actuator)
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
            file_path = get_filepath(str(values[f"-SCAN-PREFIX-"]))
            scanning_goals = []
            for i, arm_server in enumerate(arm_servers):
                scanning_goal = common.create_scanning_goal(ScanningGoal.MAP,
                                                            planning_link=str(values[f"-PLANNING-{i}-"]),
                                                            file_path=file_path)
                scanning_goal.path_constraints.volume_constraint = int(values["-CONSTRAINT-"])
                scanning_goal.path_constraints.allowed_planning_time = float(values["-PLANNING-TIME-"])
                scanning_goal.path_constraints.fix_end_effector = fix_end_effector=bool(int(values["-FEE-"]))
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
            
            mapping_goal.path_constraints.volume_constraint = int(values["-CONSTRAINT-"])
            mapping_goal.path_constraints.allowed_planning_time = float(values["-PLANNING-TIME-"])
            mapping_goal.path_constraints.fix_end_effector = fix_end_effector=bool(int(values["-FEE-"]))
            rail_pose = Point(x=float(values["-RAIL-X-"]), y=0.0, z=float(values["-RAIL-Z-"]))
            rail_pose_stamped = PointStamped(point=rail_pose)
            marker_rail_pose = Point(x=float(values["-RAIL-X-MARKER-"]), y=0.0, z=float(values["-RAIL-Z-MARKER-"]))
            marker_rail_pose_stamped = PointStamped(point=marker_rail_pose)
            mapping_goal.rail_pose = rail_pose_stamped
            mapping_goal.rail_marker_target = marker_rail_pose_stamped


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
            if archie_controller.home(home_poses=init_poses, 
                                        control_links=control_links, 
                                        rail_home=rail_home,
                                        planning_time=float(values["-PLANNING-TIME-"]),
                                        fix_end_effector=bool(int(values["-FEE-"])), 
                                        constraint_type=int(values["-CONSTRAINT-"])):
                print("Home Task Started")
            else:
                print("Home Request Failed")

        elif event == actuator:
            task_server = values['-ACTIVE-TASK-SERVER-']
            arm_planning_link = values[f"-PLANNING-{task_servers.index(values['-ACTIVE-TASK-SERVER-'])}-"]
            task_control_frame = values[f"-TASK-FRAME-{task_servers.index(values['-ACTIVE-TASK-SERVER-'])}-"]
            archie_controller.start_task(task_server=task_server, 
                                            task_control_frame=task_control_frame,
                                            arm_planning_link=arm_planning_link,
                                            task=actuator)
                    
            print(f"{actuator} Task Started: {values['-ACTIVE-TASK-SERVER-']}")
        
        elif event == 'Stop':
            archie_controller.stop()
            print("Stop Sent")
        
        elif event == 'Start Arms':
            archie_controller.start_arms(arm_servers)
        
        elif event == 'Stop Arms':
            archie_controller.stop_arms(arm_servers)
        
        elif event == 'Enable Rails':
            archie_controller.enable_rails(rail_servers)

        elif event == 'Stop Rails':
            archie_controller.stop_rails(rail_servers)

        #Update UI with state
        if "-NAV-STATE-" in window.AllKeysDict:
            navigation_status = common.goal_to_str(navigation_status)
            window['-NAV-STATE-'].update(navigation_status)
        
        rail_home = utils.create_pose_stamped_msg(0, 0, 0, "rail_frame")
        if "-RAIL-X-" in window.AllKeysDict:
            rail_home = utils.create_pose_stamped_msg(float(values["-RAIL-X-"]), 0, float(values["-RAIL-Z-"]), "rail_frame")

        if "-TASK-STATE-" in window.AllKeysDict:
            active_task_server = values["-ACTIVE-TASK-SERVER-"] if task_servers else None
            navigation_status, module_status = archie_controller.get_state(active_task_server=None)

            module_status, arm_states  = module_status

            for key in arm_servers:
                status, feedback = arm_states[key]
                status = common.goal_to_str(status)
                window[key].update(f"{status}")

            modules_idle = archie_controller.modules_idle(active_task_server=None)
            if mapping_server and not modules_idle["mapping_client"]:
                mapping_status, mapping_feedback = module_status["mapping_client"]
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
                window["-TASK-STATE-"].update(f"Mapping: {task_status} {mapping_status}")

            elif task_servers and "task_client" in modules_idle.keys() and not modules_idle["task_client"]:
                task_status, task_feedback = module_status["task_client"]
                count = task_feedback.count
                total = task_feedback.total

                if not f"-TASK-" in window.AllKeysDict:
                        window.extend_layout(window['-Column-'], task_ui(f"-TASK-"))
                
                task_status    = common.goal_to_str(task_status)
                task_status = common.task_to_str(task_feedback.status, actuator)
                actuation_status = common.actuation_to_str(task_feedback.actuation_feedback.feedback)
                window[f"-TASK-"].update(f"Actuation: {actuation_status} Count: {count}/{total}")
                window["-TASK-STATE-"].update(f"TASK: {task_status} {task_status}")
                
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
