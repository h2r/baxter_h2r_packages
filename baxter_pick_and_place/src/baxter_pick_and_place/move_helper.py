#! /usr/bin/env python

import roslib
roslib.load_manifest("baxter_pick_and_place")
import rospy
import math
import sys
import copy
import moveit_commander
import baxter_interface
import genpy
import random
import traceback


from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

class MoveHelper:

	@staticmethod
	def move_to_neutral(limb, use_joint_trajectory = False):
		if use_joint_trajectory:
			return MoveHelper._move_to_neutral_follow_joint(limb)
		arm = baxter_interface.limb.Limb(limb)
		arm.move_to_neutral()
		

	@staticmethod
	def _move_to_neutral_follow_joint(limb):
		import actionlib
		from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

		limb_command = actionlib.SimpleActionClient("/robot/" + limb + "_velocity_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
		limb_command.wait_for_server()
		arm = baxter_interface.limb.Limb(limb)

		trajectory = JointTrajectory()
		trajectory.header.stamp = rospy.Time.now()
		current_joints = arm.joint_angles()
		angles = dict(zip(arm.joint_names(),
                          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
		joints = MoveHelper.interpolate_joint_positions(current_joints, angles)
		index = 0
		for joint_dict in joints:
			point = JointTrajectoryPoint()
			point.time_from_start = rospy.rostime.Duration(0.015 * index)
			
			index += 1
			for name, angle in joint_dict.iteritems():
				if (index == 1):
					trajectory.joint_names.append(name)
				point.positions.append(angle)
			trajectory.points.append(point)
		trajectory.header.stamp = rospy.Time.now() + rospy.rostime.Duration(1.0)
		goal = FollowJointTrajectoryGoal(trajectory=trajectory)
		rospy.loginfo("Moving left arm to neutral ")
		limb_command.send_goal(goal)
		limb_command.wait_for_result()

	@staticmethod
	def interpolate_joint_positions(start, end):
		joint_arrays = []
		maxPoints = 2
		for name in start.keys():
				if name in end:
					diff = math.fabs(start[name] - end[name])
					numPoints = diff / 0.01
					if numPoints > maxPoints:
						maxPoints = int(numPoints)
		for i in range(maxPoints):
			t = float(i) / maxPoints
			joints = dict()
			for name in start.keys():
				if name in end:
					current = (1 - t)*start[name] + t * end[name]
					joints[name] = current
			joint_arrays.append(joints)
		return joint_arrays

	@staticmethod
	def set_grasps_at_pose(pose, grasps):
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

			TODO: this needs to transform a grasp based on the object's pose




			newGrasp.grasp_quality = 1.0
			correctedGrasps.append(newGrasp)

		return correctedGrasps

	@staticmethod
	def create_grasp_markers(grasps, object_name):
		return [MoveHelper.create_grasp_marker(grasp, object_name) for grasp in grasps]

	@staticmethod
	def _transpose_grasp_pose_to_marker_pose(grasp_pose):
		import tf.transformations
		import copy
		from geometry_msgs.msg import Quaternion
		
		zAxis = tf.transformations.quaternion_from_euler(0, -3.14159 / 2.0, 0)
		grasp_quat = (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w)
		result_quat = tf.transformations.quaternion_multiply(grasp_quat, zAxis)
		
		marker_pose = copy.deepcopy(grasp_pose)
		marker_pose.orientation = Quaternion(*result_quat)
		return marker_pose

	@staticmethod
	def create_grasp_marker(grasp, object_name):
		from visualization_msgs.msg import Marker

		marker = Marker()
		marker.type = 0
		marker.id = int(grasp.id)
		marker.header = grasp.grasp_pose.header
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = grasp.grasp_pose.header.frame_id
		marker.pose = MoveHelper._transpose_grasp_pose_to_marker_pose(grasp.grasp_pose.pose)
		marker.ns = object_name
		marker.lifetime.secs = 5
		marker.action = 0
		marker.color.r = 1
		marker.color.g = 1
		marker.color.b = 1
		marker.color.a = 1
		marker.scale.x = 0.1
		marker.scale.y = 0.03
		marker.scale.z = 0.03
		return marker

	@staticmethod
	def add_table(position = None, height = 1.0):
		from geometry_msgs.msg import PoseStamped
		scene = moveit_commander.PlanningSceneInterface()
		p = PoseStamped()
 		p.header.frame_id = "/base"
 		if position == None:
	   		p.pose.position.x = 0.35  
	  		p.pose.position.y = 0
	  		p.pose.position.z = -0.75
	  	else:
	  		p.pose.position = position
	  		p.pose.position.z -= height / 2.0
  		scene.add_box("table", p, (2.1, 2.0, height))#0.35