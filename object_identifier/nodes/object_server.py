#! /usr/bin/env python

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
from moveit_msgs.msg import Grasp, PlanningScene
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

default_object_timeout = rospy.Duration(5.0)
class ObjectServer:
	def __init__(self):
		self.transformer = TransformListener()
		self.objects = ["kinect", "table", "tripod", "boundary1", "boundary2"]
		self.last_time_seen = dict()
		self.markers_publisher = rospy.Publisher("/object_markers", Marker)
		
		rospy.Subscriber("/ar_objects", RecognizedObjectArray, self.markers_callback)
		rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, self.scene_callback)
		self.scene = moveit_commander.PlanningSceneInterface()
		
	def markers_callback(self, msg):
		object_poses = dict()
		self.objects = ["kinect", "table", "tripod", "boundary1", "boundary2"]
		for object in self.objects:
			self.last_time_seen[object] = rospy.Time.now()
		if len(msg.objects) == 0:
			rospy.logerr("No objects identified")
			return

		for object in msg.objects:
			self.last_time_seen[object.type.key] = rospy.Time.now()
			pose = GraspingHelper.getPoseStampedFromPoseWithCovariance(object.pose)
			self.add_object_at_pose(str(object.type.key), pose)
			object_poses[str(object.type.key)] = pose
			self.objects.append(object.type.key)
			marker = MoveHelper.create_pose_marker(pose, object.type.key, 2, 15, (1,0,0,1))
			self.markers_publisher.publish(marker)
			
	def add_object_at_pose(self, name, pose):

		width = 0.03
		length = 0.03
		height = 0.2
		if name == "spoon":
			width = 0.03
			height = 0.03
			length = 0.2

		#print("Adding " + name)
		self.scene.add_box(name, pose, (length, width, height))

	def scene_callback(self, msg):
		kinect_in_scene = False

		for object in msg.world.collision_objects:
			should_remove = False

			time_since_seen = rospy.Time.now()
			if object.id not in self.last_time_seen.keys():
				should_remove = True
			else:
				time_since_seen = rospy.Time.now() - self.last_time_seen[object.id]
				should_remove |= time_since_seen > default_object_timeout
			
			if should_remove:
				rospy.loginfo("Object " + object.id + " has not been seen in at least " + str(time_since_seen.to_sec()) + ". Removing object from MoveIt collision scene")
				#self.scene.remove_world_object(object.id)

			if object.id == "kinect":
				kinect_in_scene = True

		if not kinect_in_scene:
			MoveHelper.add_kinect(self.transformer)

if __name__=='__main__':
	rospy.init_node("object_server")
	server = ObjectServer()
	rospy.spin()