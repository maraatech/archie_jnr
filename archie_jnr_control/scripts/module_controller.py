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
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
from cares_msgs.msg import MappingAction, MappingGoal, MappingFeedback, MappingResult
from cares_msgs.msg import ArchieRailCmdAction, ArchieRailCmdGoal, ArchieRailCmdFeedback, ArchieRailCmdResult
from cares_msgs.msg import CutAction, CutGoal, CutFeedback, CutResult
from cares_msgs.msg import ThinAction, ThinGoal, ThinFeedback, ThinResult

from cares_msgs.srv import ExtractCutPoints, ExtractCutPointsRequest, ExtractThinPoints, ExtractThinPointsRequest

from cares_lib_ros.action_client import ActionClient

# Task ModuleController will be extended to carry out more than just the mapping task in the future for Archie?
class ModuleController(object):
    def __init__(self, servers, task):
        # Task Clients
        self.modules = {"task_client": None}
        self.arm_clients = {}
        self.task_clients = {}
        self.mapping_clients = {}
        for arm_server in servers["arm_servers"]:
            self.arm_clients[arm_server] = ActionClient(arm_server, PlatformGoalAction, PlatformGoalFeedback)

        self.rail_clients = {}
        if "rail_servers" in servers.keys() and servers["rail_servers"]  is not None:
            for rail_server in servers["rail_servers"]:
                self.rail_clients[rail_server] = ActionClient(rail_server, ArchieRailCmdAction, ArchieRailCmdFeedback)

        if "mapping_server" in servers.keys() and servers["mapping_server"] is not None:
            mapping_server = servers["mapping_server"]
            self.mapping_client = ActionClient(mapping_server, MappingAction, MappingFeedback)
            self.modules["mapping_client"] = self.mapping_client
        
        print(f"Servers: {servers}")
        print(f"Task: {task}")
        if "task_servers" in servers.keys() and servers["task_servers"] is not None:
            self.task_clients_indexed = []
            if task == "Cutting":
                extract_cut_points_service = rospy.get_param("~extract_cut_points_service", "vine_decision_making_server/extractCutPoints")
                self.extract_cut_points_service_proxy = rospy.ServiceProxy(extract_cut_points_service, ExtractCutPoints)
                for task_server in servers["task_servers"]:
                    self.task_clients[task_server] = ActionClient(task_server, CutAction, CutFeedback)
                    self.task_clients_indexed.append(self.task_clients[task_server])
            elif task == "Thinning":
                print("Setting up thinning task")
                extract_thin_points_service = rospy.get_param("~extract_thin_points_service", "fruitlet_decision_making_server/extractThinPoints")
                self.extract_thin_points_service_proxy = rospy.ServiceProxy(extract_thin_points_service, ExtractThinPoints)
                for task_server in servers["task_servers"]:
                    self.task_clients[task_server] = ActionClient(task_server, ThinAction, ThinFeedback)
                    self.task_clients_indexed.append(self.task_clients[task_server])

            

    def stop(self):
        if not self.modules["mapping_client"].is_idle():
            self.stop_mapping()
        elif not self.modules["task_client"].is_idle():
            self.stop_task()
        else:
            self.stop_rails()
            self.stop_arms()
    
    def stop_rails(self):
        rail_goal = ArchieRailCmdGoal()
        rail_goal.command = ArchieRailCmdGoal.STOP
        for rail_client in self.rail_clients.values():
            rail_client.send_goal(rail_goal)

    def stop_arms(self):
        self.stop_rails()
        platform_goal = PlatformGoalGoal()
        platform_goal.command = PlatformGoalGoal.STOP
        for arm_client in self.arm_clients.values():
            arm_client.send_goal(platform_goal) 

    def stop_mapping(self):
        mapping_goal = MappingGoal()
        mapping_goal.command = MappingGoal.STOP
        self.modules["mapping_client"].send_goal(mapping_goal)
        while not self.modules["mapping_client"].is_idle():
            pass
    
    def stop_task(self, task):
        if task == "Cutting":
            task_goal = CutGoal()
            task_goal.command = CutGoal.STOP
        elif task == "Thinning":
            task_goal = ThinGoal()
            task_goal.command = ThinGoal.STOP
        for task_client in self.task_clients.values():
            task_client.send_goal(task_goal)
            while not task_client.is_idle():
                pass

    def start_mapping(self, mapping_goal):
        self.mapping_client.send_goal(mapping_goal)

    def start_task(self, task_server, task_control_frame, arm_planning_link, task):
        if task == "Cutting":
            task_goal = CutGoal()
            task_goal.command = CutGoal.CUT
            task_goal.cutting_frame = task_control_frame
            task_goal.planning_frame = arm_planning_link
            task_goal.cut_points = self.extract_cut_points_service_proxy(ExtractCutPointsRequest(planning_link=arm_planning_link)).cut_points
        elif task == "Thinning":
            extract_request = ExtractThinPointsRequest()
            extract_request.planning_link = arm_planning_link

            task_goal = ThinGoal()
            task_goal.command = ThinGoal.THIN
            task_goal.thinning_frame = task_control_frame
            task_goal.planning_frame = arm_planning_link
            task_goal.fruitlets = self.extract_thin_points_service_proxy(extract_request).fruitlets.fruitlets
        
        self.task_clients[task_server].send_goal(task_goal)


    # Pass through link_id
    # Pass through link_id
    def home(self, home_poses, control_links, rail_home, planning_time, fix_end_effector, constraint_type):
        if not self.mapping_client.is_idle():
            print("Stopping Mapping")
            self.stop_mapping()
        
        for key in home_poses.keys():
            platform_goal = PlatformGoalGoal()
            platform_goal.command = PlatformGoalGoal.MOVE
            platform_goal.link_id.data = control_links[key]
            platform_goal.target_pose  = home_poses[key]
            platform_goal.path_constraints.allowed_planning_time = planning_time
            platform_goal.path_constraints.volume_constraint = constraint_type
            platform_goal.path_constraints.fix_end_effector = fix_end_effector
            self.arm_clients[key].send_goal(platform_goal)
            print(f"Moving Arm: {key}")
            while not self.arm_clients[key].is_idle():
                pass

        for key, arm_client in self.arm_clients.items():
            goal_state, _ = arm_client.get_state()
            if goal_state != GoalStatus.SUCCEEDED:
                return

        for rail_client in self.rail_clients.values():
            rail_goal = ArchieRailCmdGoal()
            rail_goal.command = ArchieRailCmdGoal.MOVE
            rail_goal.target_pose = rail_home
            rail_client.send_goal(rail_goal)

        print(f"Wiating for rails to reach home")
        for rail_client in self.rail_clients.values():
            while not rail_client.is_idle():
                pass
    
    # Extend state to return arm states as well
    # Extend state to return arm states as well
    def get_state(self, active_task_client):
        arm_states = {}
        for key, value in self.arm_clients.items():
            arm_states[key] = value.get_state()

        module_states = {}
        if active_task_client:
            self.modules["task_client"] = self.task_clients[active_task_client]
        for module_name, module in self.modules.items():
            if not active_task_client and module_name == "task_client":
                continue
            module_states[module_name] = module.get_state()
        return module_states, arm_states 

    def idle(self, active_task_client):
        idle_status = {}
        if active_task_client:
            self.modules["task_client"] = self.task_clients[active_task_client]
        for module_name, module in self.modules.items():
            if not active_task_client and module_name == "task_client":
                continue
            idle_status[module_name] = True
            if not module.is_idle():
                idle_status[module_name] = False
        return idle_status

    def get_available_modules(self):
        return self.modules.keys()


        