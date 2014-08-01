#! /usr/bin/env python

import roslib
roslib.load_manifest("baxter_grasps_server")

import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import yaml
import genpy
import copy


from threading import Thread
from grasping_helper import GraspingHelper
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.srv import GetObjectInformation
from trajectory_msgs.msg import JointTrajectoryPoint
from ar_track_alvar.msg import AlvarMarker, AlvarMarkers

from tf import TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException



class Annotator:
	def __init__(self):
		rospy.Subscriber("/ar_pose_markers", self.marker_callback)
		self.object_info = rospy.ServiceProxy('get_object_info', GetObjectInformation)
		self.transformer = TransformListener()
		self.broadcaster = TransformBroadcaster()
		self.is_annotating = False
		self.commands = GraspingHelper.get_available_commands()
		self.commands["save"] = self.write_grasps

	def marker_callback(self, msg):
		self.objects = []
		for marker in msg.markers:
			self.objects.append(marker.id)
			self.object_poses[marker.id] = marker.pose
		self.broadcast_transforms()

		if not self.is_annotating:
			self.is_annotating = True
			self.current_thread = Thread(None, self.annonate_grasps)
			self.current_thread.start()

	def annotate_grasps(self):
		object_id = GraspingHelper.get_name(self.objects)
		self.gripper = GraspingHelper.get_gripper()
		self.frame_id = self.gripper + "_gripper"

		self.grasps = []
		keep_going = True
		index = 0
		while keep_going:
			response = "continue"
			while (len(response) > 0 and response not in self.commands.keys):
				response = raw_input("Press enter to annotate the grasp or type 'save' to end annotation ")

			grasps = self.commands[response](self.gripper, self.frame_id, str(object_id), index)
			index += 1
			self.grasps.extend(grasps)
		
		
	def write_grasps(self, *args):
		GraspingHelper.write_grasps(self.grasps)

	def go(self):
		rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, self.objectsCallback, None, 10, 650000)
		rospy.spin()


if __name__=='__main__':
	rospy.init_node("annonate_grasps")
	annotator = Annotator()
	annotator.go()