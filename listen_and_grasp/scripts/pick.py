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
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface


## END_SUB_TUTORIAL

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from meldon_detection.msg import MarkerObjectArray, MarkerObject
from baxter_grasp_server.srv import GraspService

class Pick:
	def __init__(self):
		self.objects = []
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)
		self.scene = moveit_commander.PlanningSceneInterface()

	def addBoundingBox(points, name):
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
		center_x = 
		center_y = 
		center_z = 
		pose = PoseStamped()
		pose.header.frame_id = "/base"
		pose.pose.position.x = (maxX + minX) / 2.0
		pose.pose.position.y = (maxY + minY) / 2.0
		pose.pose.position.z = (maxZ + minZ) / 2.0
		self.scene.add_box(name, pose, (dim_x, dim_y, dim_z))

	def objectsCallback(msg):
		for object in self.objects:
			self.scene.remove_world_object(object)
		self.objects.clear()
		for object in msg.objects:
			self.objects.add(object.name)
			self.addBoundingBox(object.points, object.name)

	def objectRequestCallback(msg):
		if msg.data not in self.objects:
			rospy.logerr("Object " + msg.data + " is not in detected objects")
			return
		
		graspResponse = self.graspService(msg.data)
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object " + msg.data)
			return

		robot.left_arm.pick(msg.data, graspResponse.grasps)

	def pick():
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('pick')
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		scene.remove_world_object("cube")
		group = moveit_commander.MoveGroupCommander("left_arm")
		group.set_start_state_to_current_state()
		left = baxter_interface.Gripper('left')
		left.calibrate()

		display_trajectory_publisher = rospy.Publisher(
		                                  '/move_group/display_planned_path',
		                                  moveit_msgs.msg.DisplayTrajectory)
		left.open()
		rospy.sleep(1)

		p = PoseStamped()
		p.header.frame_id = "/base"
		p.pose.position.x = 0.85
		p.pose.position.y = 0.3
		p.pose.position.z = -0.3
		scene.add_box("cube", p, (0.5, 0.5, 0.5))

		p.pose.position.y = 0.5
		p.pose.position.z = -0.3
		#scene.add_box("table", p, (0.5, 1.5, 0.35))
		# pick an object

		## Planning to a Pose goal
		## ^^^^^^^^^^^^^^^^^^^^^^^
		## We can plan a motion for this group to a desired pose for the 
		## end-effector
		print "============ Generating plan 1"


		# Create grasp
		grasp_pose = PoseStamped()
		grasp_pose.header.frame_id="base"
		grasp_pose.pose.position.x = 0.6
		grasp_pose.pose.position.y = 0.4
		grasp_pose.pose.position.z = 0
		grasp_pose.pose.orientation.x = 0
		grasp_pose.pose.orientation.y = 0.707
		grasp_pose.pose.orientation.z = 0
		grasp_pose.pose.orientation.w = .707
		grasp = Grasp()

		grasp.grasp_pose = grasp_pose
		grasp.pre_grasp_approach.direction.vector.y = 0
		grasp.pre_grasp_approach.direction.vector.x = 0
		grasp.pre_grasp_approach.direction.vector.z = 1
		grasp.pre_grasp_approach.direction.header.frame_id = "base"
		grasp.pre_grasp_approach.min_distance = 0.01
		grasp.pre_grasp_approach.desired_distance = 0.25

		grasp.post_grasp_retreat.direction.header.frame_id = "base"
		grasp.pre_grasp_approach.direction.vector.y = 0
		grasp.pre_grasp_approach.direction.vector.x = 0
		grasp.pre_grasp_approach.direction.vector.z = -1
		grasp.post_grasp_retreat.min_distance = 0.01
		grasp.post_grasp_retreat.desired_distance = 0.25

		grasp.pre_grasp_posture.header.frame_id="base"
		grasp.pre_grasp_posture.joint_names.append("left_gripper_l_finger_joint")
		pre_point = JointTrajectoryPoint()
		pre_point.positions.append(0.0095)
		grasp.pre_grasp_posture.points.append(pre_point)

		grasp.grasp_posture.header.frame_id="base"
		grasp.grasp_posture.joint_names.append("left_gripper_l_finger_joint")
		point = JointTrajectoryPoint()
		point.positions.append(-0.0125)  
		grasp.grasp_posture.points.append(point)
		grasp.allowed_touch_objects.append("cube")
		grasps = []
		grasps.append(grasp)

		group.set_goal_position_tolerance(10)
		group.set_planning_time(20)

		robot.left_arm.pick("cube", grasps)

if __name__=='__main__':
	rospy.init_node("Pick object")
	rospy.Subscriber("/labeled_objects", MarkerObjectArray, objectsCallback)
	try:
		pick()
	except rospy.ROSInterruptException:
		pass
