#!/usr/bin/env python

import roslib
roslib.load_manifest("object_identifier")

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

class Pick:
	def __init__(self):
		self.transformer = TransformListener()
		
		rospy.Subscriber("/ar_objects", RecognizedObjectArray, self.markers_callback)
		self.scene = moveit_commander.PlanningSceneInterface()
		
	def add_object_at_pose(self, name, pose):
		width = 0.03
		length = 0.03
		height = 0.2
		if name == "spoon":
			width = 0.03
			height = 0.03
			length = 0.2

		#pose.pose.position.z += height / 2.0
		self.scene.add_box(name, pose, (length, width, height))

	def getPoseStampedFromPoseWithCovariance(self, pose):
		pose_stamped = PoseStamped()
		pose_stamped.header= copy.deepcopy(pose.header)
		pose_stamped.pose = copy.deepcopy(pose.pose.pose)
		now = rospy.Time.now()
		self.transformer.waitForTransform("/world", pose_stamped.header.frame_id, rospy.Time(), rospy.Duration(4,0))
		pose_stamped.header.stamp = self.transformer.getLatestCommonTime("/world", pose_stamped.header.frame_id)
		transformedPose = self.transformer.transformPose("/world", pose_stamped)
		return transformedPose
		
	def markers_callback(self, msg):

		if len(msg.objects) == 0:
			rospy.logerr("No objects identified")
			return

		for object in msg.objects:
			pose = self.getPoseStampedFromPoseWithCovariance(object.pose)
			self.add_object_at_pose(str(object.type.key), pose)		


	def go(self, args):
		moveit_commander.roscpp_initialize(args)
		rospy.spin()

if __name__=='__main__':
	rospy.init_node("Pick_object")
	pick = Pick()
	pick.go(sys.argv)