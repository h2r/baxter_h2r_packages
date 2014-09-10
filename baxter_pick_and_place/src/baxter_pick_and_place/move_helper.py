#! /usr/bin/env python

import roslib
roslib.load_manifest("baxter_pick_and_place")
import rospy

import copy
import genpy
import math
import numpy
import random
import sys
import traceback

import actionlib
import baxter_interface
import moveit_commander
import tf
import tf.transformations

from tf import TransformListener
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped, Quaternion, Pose, PoseStamped	
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

class MoveHelper:

	@staticmethod
	def move_to_neutral(limb, use_moveit = False):
		if use_moveit:
			return MoveHelper._moveit_move_to_neutral(limb)
		arm = baxter_interface.limb.Limb(limb)
		arm.move_to_neutral()
		

	@staticmethod
	def _moveit_move_to_neutral(limb):
		group = moveit_commander.MoveGroupCommander(limb + "_arm")
		group.set_named_target(limb + "_neutral")
		group.plan()
		group.go()

	#@staticmethod
	# #def _move_to_neutral_follow_joint(limb):
	# 	limb_command = actionlib.SimpleActionClient("/robot/" + limb + "_velocity_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	# 	limb_command.wait_for_server()

	# 	arm = baxter_interface.limb.Limb(limb)
		
	# 	current_angles = dict()
	# 	keep_going = True
	# 	while keep_going:
	# 		try: #because joint_angles doesn't always work???
	# 			current_angles = arm.joint_angles()
	# 			diff = -0.55 - current_angles["left_s1"] 
	# 			left_e1 = current_angles["left_e1"] - diff
	# 			left_w1 = current_angles["left_w1"] - diff
	# 			keep_going = False
	# 		except KeyError as e:
	# 			keep_going = True
	# 		except RuntimeError as e:
	# 			keep_going = True

	# 	pick_up_shoulder = MoveHelper._create_joint_angles({"left_s1" : -0.55, "left_e1":left_e1, "left_w1": left_w1}, arm)
	# 	pick_up_shoulder_trajectory = MoveHelper._create_joint_trajectory(arm, limb_command, pick_up_shoulder)

	# 	neutral_angles = MoveHelper._create_joint_angles(MoveHelper._neutral(arm), arm, pick_up_shoulder)
	# 	neutral_angles_trajectory = MoveHelper._create_joint_trajectory(arm, limb_command, neutral_angles, pick_up_shoulder)

	# 	trajectory = MoveHelper._concatenate_trajectories(pick_up_shoulder_trajectory, neutral_angles_trajectory)
	# 	MoveHelper._execute_joint_follower_trajectory(trajectory, limb_command)

	@staticmethod
	def _neutral(arm):
		return dict(zip(arm.joint_names(),
                [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))

	@staticmethod
	def _create_joint_angles(new_angles, arm, base_angles = None):
		if base_angles is None:
			base_angles = arm.joint_angles()
		angles = copy.deepcopy(base_angles)
		for joint, angle in new_angles.iteritems():
			angles[joint] = angle
		return angles

	@staticmethod
	def _concatenate_trajectories(*trajectories):
		if len(trajectories) == 0:
			return
		trajectory = copy.deepcopy(trajectories[0])
		trajectory.points = []
		index = 0
		for traj in trajectories:
			for point in traj.points:
				point.time_from_start = rospy.Duration(0.02 * index)
				trajectory.points.append(point)
				index+=1
		trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
		return trajectory

	@staticmethod
	def _create_joint_trajectory(arm, limb_command, goal_angles, start_angles = None):
		trajectory = JointTrajectory()
		trajectory.header.stamp = rospy.Time.now()

		if start_angles is None:
			start_angles = arm.joint_angles()
		
		joints = MoveHelper.interpolate_joint_positions(start_angles, goal_angles)
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
		return trajectory
		

	@staticmethod
	def _execute_joint_follower_trajectory(joint_trajectory, limb_command):
		goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory)
		limb_command.send_goal(goal)
		limb_command.wait_for_result()
		rospy.sleep(10.0)

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
	def set_grasps_at_pose(pose, grasps, transformer, object_frame_id = None):
		when = transformer.getLatestCommonTime("world", "camera_link")
		correctedGrasps = []
		index = 0
		pose.header.stamp = rospy.Time.now()
		for grasp in grasps:
			newGrasp = copy.deepcopy(grasp)
			newGrasp.id = str(index)
			index += 1
			newGrasp.pre_grasp_posture.header.stamp = when
			newGrasp.grasp_posture.header.stamp = when

			#if object_frame_id != None:
			#	newGrasp.grasp_pose.header.frame_id=object_frame_id
			#	grasp_pose = MoveHelper._get_grasp_pose_relative_to_stamped_pose(transformer, grasp.grasp_pose, pose)
			#	newGrasp.pre_grasp_approach.direction = MoveHelper._get_direction_from_pose(transformer, grasp_pose, newGrasp.pre_grasp_approach.direction)
			#else:
			newGrasp.grasp_pose = MoveHelper._get_grasp_pose_relative_to_stamped_pose(transformer, grasp.grasp_pose, pose)
			newGrasp.pre_grasp_approach.direction = MoveHelper._get_direction_from_pose(transformer, newGrasp.grasp_pose, newGrasp.pre_grasp_approach.direction)
			newGrasp.grasp_quality = 1.0
			correctedGrasps.append(newGrasp)

		return correctedGrasps

	@staticmethod
	def _get_grasp_pose_relative_to_stamped_pose(transformer, grasp_pose, pose):
		#rospy.loginfo("object pose")
		#print(str(pose))

		pose.header.stamp = transformer.getLatestCommonTime("world", "camera_link")
		if "world" != pose.header.frame_id:
			pose = transformer.transformPose("world", pose)

		#rospy.loginfo("original grasp pose")
		#print(str(grasp_pose))
		#rospy.loginfo("object pose in world")
		#print(str(pose))

		grasp_pose_transform = MoveHelper._get_transform_from_pose(grasp_pose.pose)
		pose_transform = MoveHelper._get_transform_from_pose(pose.pose)
		total_transform = tf.transformations.concatenate_matrices(pose_transform, grasp_pose_transform)
		
		#rospy.loginfo("transform")
		#print(str(total_transform))
		
		new_pose_quaternion = tf.transformations.quaternion_from_matrix(total_transform)
		scale, shear, angles, translate, perspective = tf.transformations.decompose_matrix(total_transform)
		new_grasp_pose = Pose(position=Point(*translate), orientation=Quaternion(*new_pose_quaternion))
		new_stamped_pose = PoseStamped(header=pose.header, pose=new_grasp_pose)
		#rospy.loginfo("new grasp pose")
		#print(str(new_stamped_pose))
		return new_stamped_pose

	@staticmethod
	def _get_grasp_pose_relative_to_pose(grasp_pose, pose):
		rospy.loginfo("original grasp pose")
		#print(str(grasp_pose))
		rospy.loginfo("object pose")
		#print(str(pose))
		grasp_pose_transform = MoveHelper._get_transform_from_pose(grasp_pose)
		pose_transform = MoveHelper._get_transform_from_pose(pose)
		total_transform = tf.transformations.concatenate_matrices(pose_transform, grasp_pose_transform)
		rospy.loginfo("transform")
		#print(str(total_transform))
		new_pose_quaternion = tf.transformations.quaternion_from_matrix(total_transform)
		scale, shear, angles, translate, perspective = tf.transformations.decompose_matrix(total_transform)
		new_grasp_pose = Pose(position=Point(*translate), orientation=Quaternion(*new_pose_quaternion))
		rospy.loginfo("new grasp pose")
		#print(str(new_grasp_pose))
		return new_grasp_pose

	@staticmethod
	def _get_transform_from_pose(pose):
		rotation = tf.transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
		translation = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
		return tf.transformations.concatenate_matrices(translation, rotation)

	@staticmethod
	def _get_direction_from_pose(transformer, grasp_pose, vector):
		grasp_pose_transform = MoveHelper._get_transform_from_pose(grasp_pose.pose)
		numpy_vector = numpy.array([vector.vector.x, vector.vector.y, vector.vector.z, 0.0])
		v = grasp_pose_transform.dot(numpy_vector)
		return Vector3Stamped(header=grasp_pose.header, vector=Vector3(v[0], v[1], v[2]))

	@staticmethod
	def create_grasp_markers(grasps, object_name, marker_type=0, lifetime=15, color=(1,1,1,1), scale=(0.1, 0.03, 0.03)):
		return [MoveHelper._create_grasp_marker(grasp, object_name, marker_type, lifetime, color, scale) for grasp in grasps]

	@staticmethod
	def _transpose_grasp_pose_to_marker_pose(grasp_pose):
		zAxis = tf.transformations.quaternion_from_euler(0, -3.14159 / 2.0, 0)
		grasp_quat = (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w)
		result_quat = tf.transformations.quaternion_multiply(grasp_quat, zAxis)
		
		marker_pose = copy.deepcopy(grasp_pose)
		marker_pose.orientation = Quaternion(*result_quat)
		return marker_pose

	@staticmethod
	def _create_grasp_marker(grasp, object_name, marker_type, lifetime, color, scale):	
		pose_stamped = copy.deepcopy(grasp.grasp_pose)	
		pose_stamped.pose = MoveHelper._transpose_grasp_pose_to_marker_pose(grasp.grasp_pose.pose)
		return MoveHelper.create_pose_marker(pose_stamped, object_name, marker_type, lifetime, color, scale, grasp.id)


	@staticmethod
	def create_pose_marker(pose_stamped, marker_name, marker_type=0, lifetime=15, color=(1,1,1,1), scale=(0.1, 0.03, 0.03), id=0):
		marker = Marker()
		marker.type = marker_type
		marker.id = int(id)
		marker.header = copy.deepcopy(pose_stamped.header)
		marker.header.stamp = rospy.Time.now()
		marker.pose = copy.deepcopy(pose_stamped.pose)
		marker.ns = marker_name
		marker.lifetime.secs = lifetime
		marker.action = 0
		marker.color.r = color[0]
		marker.color.g = color[1]
		marker.color.b = color[2]
		marker.color.a = color[3]
		marker.scale.x = scale[0]
		marker.scale.y = scale[1]
		marker.scale.z = scale[2]
		return marker

	@staticmethod
	def add_table(position = None, height = 0.2):
		scene = moveit_commander.PlanningSceneInterface()
		p = PoseStamped()
 		p.header.frame_id = "/base"
 		if position == None:
	   		p.pose.position.x = 0.35  
	  		p.pose.position.y = 0
	  		p.pose.position.z = -0.75
	  	else:
	  		p.pose.position = position
	  		p.pose.position.z -= height/2.0
  		scene.add_box("table", p, (1.4, 2.0, height))#0.35

  	@staticmethod
  	def add_kinect(transformer):
  		scene = moveit_commander.PlanningSceneInterface()
		p = PoseStamped()
 		p.header.frame_id = "/camera_link"
 		p.pose.orientation.w = 1.0

  		scene.add_box("kinect", p, (0.15, 0.3, 0.3))#0.35

  		p = PoseStamped()
 		p.header.frame_id = "/world"
 		p.pose.position.x = 0.9
 		p.pose.orientation.w = 1.0

  		scene.add_box("tripod", p, (0.15, 2.0, 1.0))#0.35

  		p = PoseStamped()
  		p.header.frame_id = "world"
  		p.pose.position.x = 0.5
  		p.pose.position.y = 0.7
  		p.pose.position.z = -0.45
  		p.pose.orientation.w = 1.0

  		scene.add_box("boundary1", p, (1.0, 0.4, 0.5))#0.35

  		p = PoseStamped()
  		p.header.frame_id = "world"
  		p.pose.position.x = 0
  		p.pose.position.y = 0.7
  		p.pose.position.z = -0.5
  		p.pose.orientation.w = 1.0

  		scene.add_box("boundary2", p, (0.1, 0.4, 2.0))#0.35