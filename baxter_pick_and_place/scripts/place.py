#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
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
import tf

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
from baxter_pick_and_place.move_helper import MoveHelper

from visualization_msgs.msg import Marker

class Place:
	def __init__(self):
		self.objects = []
		self.object_bounding_boxes = dict()
		self.objectPoses = dict()
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)
		self.scene = moveit_commander.PlanningSceneInterface()
		#self.robot = moveit_commander.RobotCommander()
		self.group = moveit_commander.MoveGroupCommander("left_arm")
		self.left_arm = baxter_interface.limb.Limb("left")
		
		self.limb_command = actionlib.SimpleActionClient("/robot/left_velocity_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
		self.limb_command.wait_for_server()
		self.transformer = TransformListener()
		
		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker)
		self.is_picking = False
		self.is_placing = False

	def pick(self, pose, object_name):
		self.group.detach_object()			

		graspResponse = self.graspService(object_name)
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object " + object_name)
			return

		self.group.set_planning_time(20)
		self.group.set_start_state_to_current_state()

		grasps = self.setGrasps(object_name, pose, graspResponse.grasps)
		self.publishMarkers(grasps, object_name)
		result = self.group.pick(object_name, grasps * 10)
		return result

	def addObject(self, name):
		width = 0.03
		pose = PoseStamped()
		pose.header.frame_id = "world"
		pose.pose.position.x = 0.7
		pose.pose.position.y = 0.2
		pose.pose.position.z = -0.0
		pose.pose.orientation.w = 1
		self.scene.add_box(name, pose, (width, width, 0.2))
		#self.group.attach_object(name)
		return pose

	def place(self, object_id, place_pose):
		result = self.group.place(object_id, place_pose)
		return result

	def getValidPlacePoses(self):
		place_poses = []
		for i in range(36):
			place_pose = PoseStamped()
			place_pose.header.frame_id = "world"
			place_pose.pose.position.x = 0.7
			place_pose.pose.position.y = 0.0
			place_pose.pose.position.z = 0.05
			quat = quaternion_from_euler(0, 0, i * 2.0 * math.pi / 36.0)
			rospy.loginfo(str(quat))
			place_pose.pose.orientation.x = quat[0]
			place_pose.pose.orientation.y = quat[1]
			place_pose.pose.orientation.z = quat[2]
			place_pose.pose.orientation.w = quat[3]
			place_poses.append(place_pose)
		return place_poses

	def setGrasps(self, name, pose, grasps):
		correctedGrasps = []
		index = 0
		for grasp in grasps:
			newGrasp = copy.deepcopy(grasp)
			newGrasp.id = str(index)
			index += 1
			newGrasp.pre_grasp_posture.header.stamp = rospy.Time(0)
			newGrasp.grasp_posture.header.stamp = rospy.Time(0)
			newGrasp.grasp_pose.header.frame_id = 'world'
			newGrasp.grasp_pose.pose.position.x += pose.pose.position.x
			newGrasp.grasp_pose.pose.position.y += pose.pose.position.y
			newGrasp.grasp_pose.pose.position.z += pose.pose.position.z
			newGrasp.grasp_quality = 1.0
			correctedGrasps.append(newGrasp)
		rospy.loginfo("corrected_grasps: "  + str(correctedGrasps))
		return correctedGrasps

	def publishMarkers(self, grasps, object_name):
		markers = MoveHelper.create_grasp_markers(grasps, object_name)
		for marker in markers:
			self.markers_publisher.publish(marker)

	def go(self, args):
		#moveit_commander.roscpp_initialize(args)
		#left_arm = baxter_interface.limb.Limb("left")
		#left_arm.move_to_neutral()

		object_name = "spoon"
		MoveHelper.move_to_neutral("left", True)
		self.scene.remove_world_object(object_name)

		rospy.sleep(5.0)
		pose = self.addObject(object_name)

		place_poses = self.getValidPlacePoses()
		
		pickSuccess = False
		try:
			pickSuccess = self.pick(pose, object_name)
		except Exception as e:
			traceback.print_exc()
			if isinstance(e, TypeError):
				pickSuccess = True
			else:
				raise e

		if not pickSuccess:
			rospy.logerr("Object pick up failed")
			return
		
		place_result = False
		try:
			for place_pose in place_poses:
				rospy.loginfo("Attempting to place object")
				rospy.loginfo(str(place_pose))
				if self.place(object_name, place_pose):
					break
		except Exception as e:
			traceback.print_exc()
			raise e

if __name__=='__main__':
	rospy.init_node("place")
	place = Place()
	place.go(sys.argv)