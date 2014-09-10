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

class GraspVisualizer:
	def __init__(self):
		self.transformer = TransformListener()
		
		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker)
		rospy.Subscriber("/ar_objects", RecognizedObjectArray, self.markers_callback)
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)

		
	def markers_callback(self, msg):
		object_poses = dict()

		if len(msg.objects) == 0:
			rospy.logerr("No objects identified")
			return

		for object in msg.objects:
			pose = GraspingHelper.getPoseStampedFromPoseWithCovariance(object.pose)
			object_poses[str(object.type.key)] = pose
			
		self.visualize_grasps(object_poses)

	def visualize_grasps(self, object_poses):
		for object_name, object_pose in object_poses.iteritems():
			graspResponse = self.graspService(object_name)
			if not graspResponse.success:
				rospy.logerr("No grasps were found for object " + object_name)
				return

			grasps = MoveHelper.set_grasps_at_pose(object_pose, graspResponse.grasps, self.transformer, object_pose.header.frame_id)
			self.publishMarkers(grasps, object_name)
			

	def publishMarkers(self, grasps, object_name):
		markers = MoveHelper.create_grasp_markers(grasps=grasps, object_name=object_name, lifetime=0.01)
		for marker in markers:
			self.markers_publisher.publish(marker)

	def go(self, args):
		rospy.spin()

if __name__=='__main__':
	rospy.init_node("Pick_object")
	visualizer = GraspVisualizer()
	visualizer.go(sys.argv)