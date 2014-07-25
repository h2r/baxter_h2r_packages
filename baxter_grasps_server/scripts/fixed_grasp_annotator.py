#! /usr/bin/env python

import roslib
roslib.load_manifest("baxter_grasps_server")

import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import yaml
import genpy
import copy
import math
from threading import Thread

from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectoryPoint

from tf import TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Annotator:
	def __init__(self):
		self.transformer = TransformListener()
		self.broadcaster = TransformBroadcaster()
		self.is_annotating = False

	def annotate_grasps(self):
		rospy.loginfo("Getting the name of the object")
		object_id = self.get_name()

		rospy.loginfo("Getting gripper information")
		self.gripper = self.get_gripper()

		self.frame_id = self.gripper + "_gripper"

		rospy.loginfo("Getting object pose")
		object_pose = self.get_object_pose()
		object_pose.header.frame_id = "/world"
		self.broadcast = True
		self.broadcast_object_thread =  Thread(None, self.broadcast_transform, None, ("/world", object_id, object_pose.pose.position, object_pose.pose.orientation))
		self.broadcast_object_thread.start()

		grasps = []
		keep_going = True
		index = 0
		commands = ["stahp", "line", "circle"]
		while keep_going:
			print("Move gripper to grasp pose")
			response = "continue"
			while (len(response) > 0 and response not in commands):
				response = raw_input("Press enter to annotate the grasp or type 'stahp' to end annotation ")

			if response == 'stahp':
				break
			elif response == "line":
				grasps.extend(self.get_line_grasps(object_id, index))
			elif response == "circle":
				grasps.extend(self.get_circle_grasps(object_id, index))
			else:
				pose = self.get_annonated_grasp(object_id)
				grasp = self.get_grasp(pose, index)
				grasps.append(grasp)
			index = len(grasps)
		self.broadcast = False
		keep_going = True
		while keep_going:
			try:
				filename = self.get_filename()
				self.write_grasps(grasps, filename)
				keep_going = False
			except IOError as e:
				print("Invalid filename")
				keep_going = True

	def get_line_grasps(self, object_id, start_index):
		grasps = []
		start_pose = self.get_annonated_grasp(object_id)
		response = raw_input("Move gripper to other point in line. Type 'stahp' to cancel ")
		if response !="staph":
			end_pose = self.get_annonated_grasp(object_id)
			
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
				grasp = self.get_grasp(pose, start_index + i)
				grasps.append(grasp)
		return grasps

	def get_circle_grasps(self, object_id, start_index):
		grasps = []
		start_pose = self.get_annonated_grasp(object_id)
		roll, pitch, yaw = self.get_array_from_quaternion(start_pose.pose.orientation)
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

			dX = -dist * math.cos(newYaw)
			dY = -dist * math.sin(newYaw)
			pose.pose.orientation = self.get_quaternion_from_array(newQuat)
			pose.pose.position.x += dX
			pose.pose.position.y += dY
			grasp = self.get_grasp(pose, start_index + i)
			grasps.append(grasp)
		return grasps

	def get_array_from_quaternion(self, quaternion):
		arry = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
		return euler_from_quaternion(arry)

	def get_quaternion_from_array(self, arry):
		return Quaternion(x=arry[0], y=arry[1], z=arry[2], w=arry[3])

	def get_object_pose(self):
		print("Move gripper to center of object")
		raw_input("Press enter when gripper is positioned")
		pose = PoseStamped()
		pose.header.frame_id = "left_gripper"
		pose.pose.position.z = 0.05
		pose.pose.orientation.w = 1.0

		self.transformer.waitForTransform("/world", pose.header.frame_id, rospy.Time(), rospy.Duration(4,0))
		pose.header.stamp = self.transformer.getLatestCommonTime("/world", pose.header.frame_id)
		transformed_pose = self.transformer.transformPose("/world", pose)
		rospy.loginfo(str(transformed_pose))
		return transformed_pose

	def broadcast_transform(self, parent, child, origin, orientation):
		while(self.broadcast):
			self.broadcaster.sendTransform((origin.x, origin.y, origin.z), (orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), child, parent)
			rospy.sleep(0.01)

	def broadcast_transforms(self):
		for object, pose in self.object_poses.iteritems():
			origin = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
			orientation = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
			self.broadcaster.sendTransform(origin, orientation, pose.header.stamp, str(object), pose.header.frame_id)

	def getPoseStampedFromPoseWithCovariance(self, pose):
		pose_stamped = PoseStamped()
		pose_stamped.header= copy.deepcopy(pose.header)
		pose_stamped.pose.position = copy.deepcopy(pose.pose.pose.position)
		pose_stamped.pose.position.z -= 0
		pose_stamped.pose.orientation = copy.deepcopy(pose.pose.pose.orientation)
		now = rospy.Time.now()
		self.transformer.waitForTransform("/world", pose_stamped.header.frame_id, rospy.Time(), rospy.Duration(4,0))
		pose_stamped.header.stamp = self.transformer.getLatestCommonTime("/world", pose_stamped.header.frame_id)
		transformedPose = self.transformer.transformPose("/world", pose_stamped)
		return transformedPose
	

	def get_name(self):
		name = ""
		while name == "":
			name = raw_input("Enter the name of the object: ")
		return name 

	def get_gripper(self):
		gripper = raw_input("'left' or 'right' ")
		while gripper != 'left' and gripper != 'right':
			gripper = raw_input("'left' or 'right' ")
		return gripper 

	
	def get_annonated_grasp(self, object):
		when = self.transformer.getLatestCommonTime(self.frame_id, str(object))
		transform = self.transformer.lookupTransform(self.frame_id, str(object), when)
		grasp_pose = PoseStamped()
		grasp_pose.pose.position = Point(transform[0][0], transform[0][1], transform[0][2])
		grasp_pose.pose.orientation = Quaternion(transform[1][0], transform[1][1], transform[1][2], transform[1][3])
		grasp_pose.header.frame_id = self.frame_id
		return grasp_pose

	def get_grasp(self, grasp_pose, id):
		grasp = Grasp()
		grasp.grasp_pose = grasp_pose
		joint_name = self.gripper + '_gripper_l_finger_joint'
		
		point = JointTrajectoryPoint()
		point.positions.append(0.095)
		grasp.pre_grasp_posture.joint_names.append(joint_name)
		grasp.pre_grasp_posture.points.append(point)

		point = JointTrajectoryPoint()
		point.positions.append(-0.0125)
		grasp.grasp_posture.joint_names.append(joint_name)

		grasp.grasp_posture.points.append(point)
		grasp.grasp_quality = 1.0
		grasp.id = str(id)
		
		grasp.post_place_retreat.desired_distance = 0.3
		grasp.post_place_retreat.min_distance = 0.01
		grasp.post_place_retreat.direction.header.frame_id = 'world'
		grasp.post_place_retreat.direction.vector.z = 1.0
		
		grasp.pre_grasp_approach.desired_distance = 0.3
		grasp.pre_grasp_approach.min_distance = 0.01
		grasp.pre_grasp_approach.direction.header.frame_id = self.gripper + "_gripper"
		grasp.pre_grasp_approach.direction.vector.z = 1.0

		grasp.post_grasp_retreat = grasp.post_place_retreat
		return grasp

	def get_filename(self):
		return raw_input("Where would you like to write the grasps to?")

	def write_grasps(self, grasps, filename):
		stream = file(filename, 'w')
		args = []
		for grasp in grasps:
			str = genpy.message.strify_message(grasp)
			args.append(yaml.load(str))

		yaml.dump(args, stream)
		print(yaml.dump(args))
		self.is_annotating = False


	def go(self):
		self.annotate_grasps()

if __name__=='__main__':
	rospy.init_node("annonate_grasps")
	annotator = Annotator()
	annotator.go()