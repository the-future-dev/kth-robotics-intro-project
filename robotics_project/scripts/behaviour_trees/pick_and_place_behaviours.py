#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseStamped
#from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from robotics_project.srv import MoveHead
from actionlib import SimpleActionClient
from std_srvs.srv import Empty

class DetectCube(pt.behaviour.Behaviour):
    """Detects the cube using ArUco marker detection"""
    
    def __init__(self):
        super(DetectCube, self).__init__("Detect Cube")
        self.detected_pose = None
        
    def setup(self, timeout):
        self.aruco_subscriber = rospy.Subscriber(
            '/aruco_single/pose', 
            PoseStamped,
            self.aruco_callback
        )
        return True
        
    def aruco_callback(self, msg):
        self.detected_pose = msg
        
    def update(self):
        if self.detected_pose is None:
            return pt.common.Status.RUNNING
        return pt.common.Status.SUCCESS

class PickCube(pt.behaviour.Behaviour):
    """Executes the pick action"""
    
    def __init__(self):
        super(PickCube, self).__init__("Pick Cube")
        self.pick_client = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        
    def setup(self, timeout):
        return self.pick_client.wait_for_server(timeout=rospy.Duration(timeout))
        
    def update(self):
        if not self.detected_pose:
            return pt.common.Status.FAILURE
            
        goal = PickUpPoseGoal()
        goal.object_pose = self.detected_pose
        
        self.pick_client.send_goal(goal)
        self.pick_client.wait_for_result()
        
        result = self.pick_client.get_result()
        if result.error_code == 1:  # SUCCESS
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE

class PlaceCube(pt.behaviour.Behaviour):
    """Executes the place action"""
    
    def __init__(self):
        super(PlaceCube, self).__init__("Place Cube")
        self.place_client = SimpleActionClient('/place_pose', PickUpPoseAction)
        
    def setup(self, timeout):
        return self.place_client.wait_for_server(timeout=rospy.Duration(timeout))
        
    def update(self):
        goal = PickUpPoseGoal()
        # Set place pose slightly above current position
        goal.object_pose = self.detected_pose
        goal.object_pose.pose.position.z += 0.05
        
        self.place_client.send_goal(goal)
        self.place_client.wait_for_result()
        
        result = self.place_client.get_result()
        if result.error_code == 1:  # SUCCESS
            return pt.common.Status.SUCCESS
        return pt.common.Status.FAILURE
