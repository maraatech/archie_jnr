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

from cares_msgs.srv import ExtractCutPoints, ExtractCutPointsRequest

from cares_lib_ros.action_client import ActionClient

# Task ModuleController will be extended to carry out more than just the mapping task in the future for Archie?
class ModuleController(object):
    def __init__(self, servers):
        # Task Clients
        self.modules = {}
        self.arm_clients = {}
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
        if "cutting_server" in servers.keys() and servers["cutting_server"] is not None:
            cutting_server = servers["cutting_server"]
            extract_cut_points_service = rospy.get_param("~extract_cut_points_service", "vine_decision_making_server/extractCutPoints")
            self.cutting_client = ActionClient(cutting_server, CutAction, CutFeedback)
            self.extract_cut_points_service_proxy = rospy.ServiceProxy(extract_cut_points_service, ExtractCutPoints)
            self.modules["cutting_client"] = self.cutting_client

    def stop(self):
        if not self.modules["mapping_client"].is_idle():
            self.stop_mapping()
        elif not self.modules["cutting_client"].is_idle():
            self.stop_cutting()
        else:
            self.stop_rails()
            self.stop_arms()
    
    def stop_rails(self):
        rail_goal = ArchieRailCmdGoal()
        rail_goal.command = ArchieRailCmdGoal.STOP
        for rail_client in self.rail_clients.values():
            rail_client.send_goal(rail_goal)

    def stop_arms(self):
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
    
    def stop_cutting(self):
        cutting_goal = CutGoal()
        cutting_goal.command = CutGoal.STOP
        self.modules["cutting_client"].send_goal(cutting_goal)
        while not self.modules["cutting_client"].is_idle():
            pass

    def start_mapping(self, mapping_goal):
        self.mapping_client.send_goal(mapping_goal)

    def start_cutting(self):
        cutting_goal = CutGoal()
        cutting_goal.command = CutGoal.CUT
        cutting_goal.cut_points = self.extract_cut_points_service_proxy(ExtractCutPointsRequest()).cut_points

        self.cutting_client.send_goal(cutting_goal)


    # Pass through link_id
    def home(self, home_poses, control_links, rail_home):
        if not self.mapping_client.is_idle():
            print("Stopping Mapping")
            self.stop_mapping()
        elif not self.modules["cutting_client"].is_idle():
            print("Stopping Cutting")
            self.stop_cutting
        
        for key in home_poses.keys():
            platform_goal = PlatformGoalGoal()
            platform_goal.command = PlatformGoalGoal.MOVE
            platform_goal.link_id.data = control_links[key]
            platform_goal.target_pose  = home_poses[key]
            self.arm_clients[key].send_goal(platform_goal)
            print(f"Moving Arm: {key}")
            while not self.arm_clients[key].is_idle():
                pass

        for key, arm_client in self.arm_clients.items():
            goal_state, _ = arm_client.get_state()
            if goal_state != GoalStatus.SUCCEEDED:
                print(f"{key} failed to home not moving rail!")
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
    def get_state(self):
        arm_states = {}
        for key, value in self.arm_clients.items():
            arm_states[key] = value.get_state()

        module_states = {}
        for module_name, module in self.modules.items():
            module_states[module_name] = module.get_state()
        return module_states, arm_states 

    def idle(self):
        idle_status = {}
        for module_name, module in self.modules.items():
            idle_status[module_name] = True
            if not module.is_idle():
                idle_status[module_name] = False
        return idle_status

    def get_available_modules(self):
        return self.modules.keys()


        