#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#

import roslib
roslib.load_manifest("baxter_pick_and_place")

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import genpy
import random
import traceback
import math
import actionlib

## END_SUB_TUTORIAL

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from moveit_msgs.msg import Grasp
from object_recognition_msgs.msg import RecognizedObjectArray
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
#from meldon_detection.msg import MarkerObjectArray, MarkerObject
from baxter_grasps_server.srv import GraspService
from ar_track_alvar.msg import AlvarMarker, AlvarMarkers
from threading import Thread
from visualization_msgs.msg import Marker
from baxter_pick_and_place.move_helper import MoveHelper
from baxter_grasps_server.grasping_helper import GraspingHelper

class Pick:
	def __init__(self):
		self.transformer = TransformListener()
		self.time_exited = rospy.Time.now()
		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker)
		self.is_picking = False
		self.is_placing = False
		rospy.Subscriber("/ar_objects", RecognizedObjectArray, self.markers_callback)
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("left_arm")
		self.group.set_workspace([0.0, -0.2, -0.30, 0.9, 1.0, 2.0] )
		self.left_arm = baxter_interface.limb.Limb("left")
		self.limb_command = actionlib.SimpleActionClient("/robot/left_velocity_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
		self.limb_command.wait_for_server()

	def is_picking_or_placing(self):
		return self.is_picking or self.is_placing
		
	def markers_callback(self, msg):
		object_poses = dict()

		if len(msg.objects) == 0:
			rospy.logerr("No objects identified")
			return

		rospy.loginfo("updating objects")
		
		for object in msg.objects:
			pose = GraspingHelper.getPoseStampedFromPoseWithCovariance(object.pose)
			object_poses[str(object.type.key)] = pose
			
		if rospy.Time.now() - self.time_exited > rospy.Duration(10.0):
			self.pick_and_place(msg.objects, object_poses)
			self.time_exited = rospy.Time.now()
		rospy.loginfo("updating objects")
		

	def pick_and_place(self, objects, object_poses):
		self.is_picking = True
		
		grasps = None
		object_name = ""
		random.shuffle(objects)

		for object in objects:
			object_name = object.type.key
			rospy.loginfo("Getting grasp for object " + object_name)
			graspResponse = self.graspService(object_name)
			if graspResponse.success:
				rospy.loginfo("grasps were found for object " + object_name)
				grasps = graspResponse.grasps
				break
		if grasps is None:
			rospy.logerr("Failed to find any grasps for the objects identified")
			self.is_picking = False
			return

		rospy.loginfo("Finding a valid place pose")
		place_poses = self.getValidPlacePoses()

		rospy.loginfo("Attempting to pick up object " + object_name)
		MoveHelper.move_to_neutral("left", True)

		pickSuccess = False
		try:
			pickSuccess = self.pick(object_poses[object_name], object_name)
		except Exception as e:
			traceback.print_exc()
			#if isinstance(e, TypeError):
			#	pickSuccess = True
			#else:
			raise e
		finally:
			self.is_placing = pickSuccess
			self.is_picking = False

		if not pickSuccess:
			rospy.logerr("Object pick up failed")
			return
		
		place_result = False
		try:
			for place_pose in place_poses:
				rospy.loginfo("Attempting to place object")
				if self.place(object_name, object_poses[object_name], place_pose):
					break
		except Exception as e:
			traceback.print_exc()
			raise e
		finally:
			self.is_placing = False

	def pick(self, object_pose, object_name):
		self.group.detach_object()			

		graspResponse = self.graspService(object_name)
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object " + object_name)
			return

		self.group.set_planning_time(20)
		self.group.set_start_state_to_current_state()

		grasps = MoveHelper.set_grasps_at_pose(object_pose, graspResponse.grasps, self.transformer, object_pose.header.frame_id)
		#print(str(grasps))
		self.publishMarkers(grasps, object_name)
		
		for grasp in grasps:
			self.group.set_pose_target(grasp.grasp_pose)
			self.group.plan()
			self.group.go()
		

		#result = self.group.pick(object_name, grasps * 5)
		#return result

	def place(self, object_id, original_pose, place_pose):
		goal_pose = copy.deepcopy(original_pose)
		goal_pose.pose.position.x = place_pose.pose.position.x
		goal_pose.pose.position.y = place_pose.pose.position.y
		result = self.group.place(object_id, goal_pose)
		return result

	def getValidPlacePoses(self):
		random_place = [random.randint(0,100), random.randint(0,100)]
		place_poses = []
		for i in range(36):	
			place_pose = PoseStamped()
			place_pose.header.frame_id = "world"
			place_pose.pose.position.x = 0.7 + random_place[0] / 1000.0 #convert back to meters
			place_pose.pose.position.y = 0.3 + random_place[1] / 1000.0
			quat = quaternion_from_euler(0, math.pi/2.0, i * 2.0 * math.pi / 36.0)
			place_pose.pose.orientation.x = quat[0]
			place_pose.pose.orientation.y = quat[1]
			place_pose.pose.orientation.z = quat[2]
			place_pose.pose.orientation.w = quat[3]
			place_poses.append(place_pose)
		return place_poses

	def publishMarkers(self, grasps, object_name):
		markers = MoveHelper.create_grasp_markers(grasps, object_name)
		for marker in markers:
			self.markers_publisher.publish(marker)

	def go(self, args):
		moveit_commander.roscpp_initialize(args)
		MoveHelper.move_to_neutral("left", True)
		rospy.sleep(5.0)
		rospy.spin()

if __name__=='__main__':
	rospy.init_node("Pick_object")
	pick = Pick()
	pick.go(sys.argv)