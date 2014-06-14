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
roslib.load_manifest("listen_and_grasp")

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import genpy


## END_SUB_TUTORIAL

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from object_recognition_msgs.msg import RecognizedObjectArray
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
#from meldon_detection.msg import MarkerObjectArray, MarkerObject
from baxter_grasps_server.srv import GraspService

from visualization_msgs.msg import Marker

class Pick:
	def __init__(self):
		self.objects = []
		self.objectPoses = dict()
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.robot = moveit_commander.RobotCommander()
		self.group = moveit_commander.MoveGroupCommander("left_arm")
		self.left_arm = baxter_interface.limb.Limb("left")
		
		self.transformer = TransformListener()
		
		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker)
		

	def addBoundingBox(self, points, name):
		minX = sys.float_info.max
		minY = sys.float_info.max
		minZ = sys.float_info.max
		maxX = -sys.float_info.max
		maxY = -sys.float_info.max
		maxZ = -sys.float_info.max

		for point in points:
			if (point.x() > maxX):
				maxX = point.x()
			if (point.y() > maxY):
				maxY = point.y()
			if (point.z() > maxZ):
				maxZ = point.z()
			if (point.x() < minX):
				minX = point.x()
			if (point.y() < minY):
				minY = point.y()
			if (point.z() < minZ):
				minZ = point.z()
		dim_x = maxX - minX
		dim_y = maxY - minY
		dim_z = maxZ - minZ

		pose = PoseStamped()
		pose.header.frame_id = "/base"
		pose.pose.position.x = (maxX + minX) / 2.0
		pose.pose.position.y = (maxY + minY) / 2.0
		pose.pose.position.z = (maxZ + minZ) / 2.0
		self.scene.add_box(name, pose, (dim_x, dim_y, dim_z))

	def addBoundingBoxAtPose(self, name):
		pose = self.objectPoses[name]
		pose.pose.position.z += 0.1
		self.scene.add_box(name, pose, (0.02, 0.02, 0.2))

	def getPoseStampedFromPoseWithCovariance(self, pose):
		pose_stamped = PoseStamped()
		pose_stamped.header= copy.deepcopy(pose.header)
		pose_stamped.pose.position = copy.deepcopy(pose.pose.pose.position)
		pose_stamped.pose.position.z -= 0
		pose_stamped.pose.orientation = copy.deepcopy(pose.pose.pose.orientation)
		now = rospy.Time.now()
		self.transformer.waitForTransform("world", pose_stamped.header.frame_id, now, rospy.Duration(4.0))
		t = self.transformer.getLatestCommonTime("world", pose_stamped.header.frame_id)
		pose_stamped.header.stamp = t
		transformedPose = self.transformer.transformPose("world", pose_stamped)
		return transformedPose
		

	def objectsCallback(self, msg):
		for object in self.objects:
			self.scene.remove_world_object(object)
		self.objects = []
		self.objectPoses = dict()
		for object in msg.objects:
			self.objects.append(object.type.key)
			newPose = self.getPoseStampedFromPoseWithCovariance(object.pose)
			self.objectPoses[object.type.key] = newPose
			self.addBoundingBoxAtPose(object.type.key)
			self.pick(object.type.key)
			self.place(object.type.key)

	def objectRequestCallback(self, msg):
		if msg.data not in self.objects:
			rospy.logerr("Object " + msg.data + " is not in detected objects")
			return
		
		graspResponse = self.graspService(msg.data)
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object " + msg.data)
			return
		self.group.set_start_state_to_current_state()
		robot.left_arm.pick(msg.data, graspResponse.grasps)

	def addTable(self):
		scene = moveit_commander.PlanningSceneInterface()
		p = PoseStamped()
 		p.header.frame_id = "/base"
   		p.pose.position.x = 0.85  
  		p.pose.position.y = 0.5
  		p.pose.position.z = -0.3
  		scene.add_box("table", p, (0.5, 1.5, 0.35))

	def pick(self, obj):
		for object in self.objects:
			self.group.detach_object(object)

		graspResponse = self.graspService("coconut")
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object coconut")
			return
		#self.addCylinder(0.8, 0, 0.3, 0.01, 0.01, 0.01)
		self.publishMarkers(graspResponse.grasps, "coconut")
		self.group.set_planning_time(20)
		self.group.set_start_state_to_current_state()
		grasps = self.setGrasps(obj, graspResponse.grasps)
		self.group.pick(obj, grasps)

	def place(self, obj):
		goal_pose = copy.deepcopy(self.objectPoses[obj])
		goal_pose.pose.position.y += 0.1
		self.group.place(obj, goal_pose)

	def setGrasps(self, name, grasps):
		pose = self.objectPoses[name]

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

		return correctedGrasps

	def publishMarkers(self, grasps, object_name):
		for grasp in grasps:
			marker = self.getMarker(grasp, object_name)
			self.markers_publisher.publish(marker)
		

	def getMarker(self, grasp, object_name):
		marker = Marker()
		marker.id = int(grasp.id)
		marker.header = grasp.grasp_pose.header
		marker.pose = grasp.grasp_pose.pose
		marker.ns = object_name + "_grasp_"
		marker.lifetime.secs = 10.0
		marker.action = 0
		marker.color.r = 1
		marker.color.g = 1
		marker.color.b = 1
		marker.color.a = 1
		marker.scale.x = 1
		marker.scale.y = 1
		marker.scale.z = 1
		return marker


	def go(self, args):
		#self.left_arm.move_to_neutral()

		moveit_commander.roscpp_initialize(args)
		rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.objectsCallback, None, 1)
		#rospy.Subscriber("/labeled_objects", MarkerObjectArray, objectsCallback)
		
		#rospy.Service('/pick_place_server', String, objectRequestCallback)
		#self.pick()
		self.addTable()
		rospy.spin()

if __name__=='__main__':
	rospy.init_node("Pick_object")
	pick = Pick()
	pick.go(sys.argv)