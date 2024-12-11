#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseStamped
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from robotics_project.srv import MoveHead
from actionlib import SimpleActionClient
from std_srvs.srv import Empty
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class DetectCube(pt.behaviour.Behaviour):
    def __init__(self):
        super(DetectCube, self).__init__("Detect Cube")
        self.detected_pose = None
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        
    def setup(self, timeout):
        self.aruco_subscriber = rospy.Subscriber(
            '/aruco_single/pose', 
            PoseStamped,
            self.aruco_callback
        )
        return True
        
    def aruco_callback(self, msg):
        try:
            # Transform pose to base_footprint frame
            transform = self.tfBuffer.lookup_transform(
                "base_footprint",
                msg.header.frame_id,
                rospy.Time(0)
            )
            self.detected_pose = do_transform_pose(msg, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")
            
    def update(self):
        if self.detected_pose is None:
            rospy.loginfo("Waiting for cube detection...")
            return pt.common.Status.RUNNING
        rospy.loginfo("Cube detected!")
        return pt.common.Status.SUCCESS

class NavigateToPosition(pt.behaviour.Behaviour):
    def __init__(self, position_name, x, y, theta):
        super(NavigateToPosition, self).__init__(f"Navigate to {position_name}")
        self.position_name = position_name
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self.cmd_vel_pub = rospy.Publisher('/key_vel', Twist, queue_size=1)
        
    def update(self):
        # Simple navigation using position-based control
        # In real implementation, should use move_base or similar
        msg = Twist()
        msg.linear.x = self.target_x
        msg.angular.z = self.target_theta
        self.cmd_vel_pub.publish(msg)
        return pt.common.Status.SUCCESS

class PickCube(pt.behaviour.Behaviour):
    def __init__(self):
        super(PickCube, self).__init__("Pick Cube")
        self.pick_client = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        
    def setup(self, timeout):
        return self.pick_client.wait_for_server(timeout=rospy.Duration(timeout))
        
    def update(self):
        if not hasattr(self.parent, 'detected_pose'):
            rospy.logerr("No cube pose available!")
            return pt.common.Status.FAILURE
            
        goal = PickUpPoseGoal()
        goal.object_pose = self.parent.detected_pose
        
        rospy.loginfo("Executing pick action...")
        self.pick_client.send_goal(goal)
        self.pick_client.wait_for_result()
        
        result = self.pick_client.get_result()
        if result.error_code == 1:
            rospy.loginfo("Pick successful!")
            return pt.common.Status.SUCCESS
        rospy.logerr("Pick failed!")
        return pt.common.Status.FAILURE

class PlaceCube(pt.behaviour.Behaviour):
    def __init__(self):
        super(PlaceCube, self).__init__("Place Cube")
        self.place_client = SimpleActionClient('/place_pose', PickUpPoseAction)
        
    def setup(self, timeout):
        return self.place_client.wait_for_server(timeout=rospy.Duration(timeout))
        
    def update(self):
        goal = PickUpPoseGoal()
        # Get place pose from parameters
        place_pose = rospy.get_param("~place_pose")
        goal.object_pose = self._create_pose_stamped(place_pose)
        
        rospy.loginfo("Executing place action...")
        self.place_client.send_goal(goal)
        self.place_client.wait_for_result()
        
        result = self.place_client.get_result()
        if result.error_code == 1:
            rospy.loginfo("Place successful!")
            return pt.common.Status.SUCCESS
        rospy.logerr("Place failed!")
        return pt.common.Status.FAILURE
        
    def _create_pose_stamped(self, pose_array):
        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pose_array[0]
        pose.pose.position.y = pose_array[1]
        pose.pose.position.z = pose_array[2]
        pose.pose.orientation.x = pose_array[3]
        pose.pose.orientation.y = pose_array[4]
        pose.pose.orientation.z = pose_array[5]
        pose.pose.orientation.w = pose_array[6]
        return pose
