#! /usr/bin/env python

import roslib
roslib.load_manifest("baxter_grasps_server")
import rospy
import copy
import genpy
import yaml

from geometry_msgs.msg import PoseStamped, Quaternion
from tf import TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class GraspingHelper:

	@staticmethod
	def get_available_commands():
		return {
			"": GraspingHelper.get_annotated_grasp_pose,
			"circle":GraspingHelper.get_circle_grasps,
			"line":GraspingHelper.get_line_grasps
		}
	
	@staticmethod
	def getPoseStampedFromPoseWithCovariance(pose):
		pose_stamped = PoseStamped()
		pose_stamped.header= copy.deepcopy(pose.header)
		pose_stamped.pose.position = copy.deepcopy(pose.pose.pose.position)
		pose_stamped.pose.orientation = copy.deepcopy(pose.pose.pose.orientation)
		return pose_stamped

	@staticmethod
	def get_gripper(self):
		gripper = raw_input("'left' or 'right' ")
		while gripper != 'left' and gripper != 'right':
			gripper = raw_input("'left' or 'right' ")
		return gripper 

	@staticmethod
	def get_name(objects):
		index = 0
		for object in objects:
			print(str(index) + ") " + self.object_lookup[object] + " (" + str(object) + ")")
			index = index + 1
		keep_going = True
		chosen_object = -1
		while keep_going:
			try:
				value = raw_input("Choose a valid number from 0 to " + str(len(objects)-1))
				chosen_object = int(value)
				keep_going = chosen_object < 0 or chosen_object >= len(objects)
			except ValueError as e:
				keep_going = True
			
		return objects[chosen_object]

	@staticmethod
	def get_annotated_grasp(gripper, gripper_frame_id, object_frame_id, index):
		grasp_pose = GraspingHelper.get_annotated_grasp_pose(gripper_frame_id, object_frame_id)
		return GraspingHelper.get_grasp_from_pose(grasp_pose, gripper, str(index))

	@staticmethod
	def get_annotated_grasp_pose(gripper_frame_id, object_frame_id):
		transformer = TransformListener()
		when = transformer.getLatestCommonTime(gripper_frame_id, object_frame_id)
		transform = transformer.lookupTransform(gripper_frame_id, object_frame_id, when)
		grasp_pose = PoseStamped()
		grasp_pose.pose.position = Point(transform[0][0], transform[0][1], transform[0][2])
		grasp_pose.pose.orientation = Quaternion(transform[1][0], transform[1][1], transform[1][2], transform[1][3])
		grasp_pose.header.frame_id = gripper_frame_id
		return grasp_pose

	@staticmethod
	def get_line_grasps(gripper, gripper_frame_id, object_frame_id, start_index):
		grasps = []
		start_pose = GraspingHelper.get_annotated_grasp_pose(object_id)
		response = raw_input("Move gripper to other point in line. Type 'stahp' to cancel ")
		if response !="staph":
			end_pose = GraspingHelper.get_annotated_grasp_pose(object_id)
			
			num_grasps = -1
			while num_grasps == -1:
				num_response = raw_input("Input number of grasps to generate: ")
				try:
					num_grasps = int(num_response)
				except ValueError as e:
					num_grasps = -1

			for i in range(num_grasps):
				pose = copy.deepcopy(start_pose)
				t = float(i) / num_grasps	
				pose.pose.position.x = (1 - t) * start_pose.pose.position.x + t * end_pose.pose.position.x
				pose.pose.position.y = (1 - t) * start_pose.pose.position.y + t * end_pose.pose.position.y
				pose.pose.position.z = (1 - t) * start_pose.pose.position.z + t * end_pose.pose.position.z
				grasp = GraspingHelper.get_grasp_from_pose(pose, gripper, start_index + i)
				grasps.append(grasp)
		return grasps

	@staticmethod
	def get_circle_grasps(gripper, gripper_frame_id, object_frame_id, start_index):
		grasps = []
		start_pose = GraspingHelper.get_annotated_grasp_pose(object_id)
		roll, pitch, yaw = GraspingHelper.get_array_from_quaternion(start_pose.pose.orientation)
		num_grasps = -1
		while num_grasps == -1:
			num_response = raw_input("Input number of grasps to generate: ")
			try:
				num_grasps = int(num_response)
			except ValueError as e:
				num_grasps = -1

		x = start_pose.pose.position.x
		y = start_pose.pose.position.y
		dist = math.sqrt(x*x + y*y)
		for i in range(num_grasps):
			pose = copy.deepcopy(start_pose)
			newYaw = yaw + float(i) * 2 * math.pi / num_grasps
			newQuat = quaternion_from_euler(roll, pitch, newYaw)
			rospy.loginfo("yaw: " + str(newYaw))
			dX = dist * math.cos(newYaw)
			dY = dist * math.sin(newYaw)
			pose.pose.orientation = self.get_quaternion_from_array(newQuat)
			pose.pose.position.x = dX
			pose.pose.position.y = dY
			grasp = GraspingHelper.get_grasp_from_pose(gripper, pose, start_index + i)
			grasps.append(grasp)
		return grasps

	@staticmethod
	def get_grasp_from_pose(grasp_pose, gripper, grasp_id):
		grasp = Grasp()
		grasp.grasp_pose = grasp_pose
		joint_name = gripper + '_gripper_l_finger_joint'
		
		point = JointTrajectoryPoint()
		point.positions.append(0.095)
		grasp.pre_grasp_posture.joint_names.append(joint_name)
		grasp.pre_grasp_posture.points.append(point)

		point = JointTrajectoryPoint()
		point.positions.append(-0.0125)
		grasp.grasp_posture.joint_names.append(joint_name)

		grasp.grasp_posture.points.append(point)
		grasp.grasp_quality = 1.0
		grasp.id = str(grasp_id)
		
		grasp.post_place_retreat.desired_distance = 0.3
		grasp.post_place_retreat.min_distance = 0.01
		grasp.post_place_retreat.direction.header.frame_id = 'world'
		grasp.post_place_retreat.direction.vector.z = 1.0
		
		grasp.pre_grasp_approach.desired_distance = 0.3
		grasp.pre_grasp_approach.min_distance = 0.01
		grasp.pre_grasp_approach.direction.header.frame_id = gripper + "_gripper"
		grasp.pre_grasp_approach.direction.vector.z = 1.0

		grasp.post_grasp_retreat = grasp.post_place_retreat
		return grasp

	@staticmethod
	def get_filename():
		return raw_input("Where would you like to write the grasps to?")

	@staticmethod
	def write_grasps(grasps):
		
		keep_going = True
		while keep_going:
			try:
				filename = GraspingHelper.get_filename()
				
				stream = file(filename, 'w')
				args = []
				for grasp in grasps:
					str = genpy.message.strify_message(grasp)
					args.append(yaml.load(str))

				yaml.dump(args, stream)
				print(yaml.dump(args))

				keep_going = False
			except IOError as e:
				print("Invalid filename")
				keep_going = True


		

	@staticmethod
	def get_array_from_quaternion(self, quaternion):
		arry = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
		return euler_from_quaternion(arry)

	@staticmethod
	def get_quaternion_from_array(self, arry):
		return Quaternion(*arry)