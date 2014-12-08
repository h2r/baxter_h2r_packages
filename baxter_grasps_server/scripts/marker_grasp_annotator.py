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
from baxter_grasps_server.grasping_helper import GraspingHelper
from baxter_pick_and_place.move_helper import MoveHelper
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.srv import GetObjectInformation
from trajectory_msgs.msg import JointTrajectoryPoint
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from visualization_msgs.msg import Marker

from tf import TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException



class Annotator:
	def __init__(self):
		self.broadcaster = TransformBroadcaster()
                topic = "/publish_detections_center/blue_labeled_objects" # node
                #topic = "/ar_objects" # ar tags
		rospy.Subscriber(topic, RecognizedObjectArray, self.object_callback)
		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker, queue_size=10)
		self.object_info = rospy.ServiceProxy('get_object_info', GetObjectInformation)
		self.transformer = TransformListener(True, 
                                                     rospy.Duration(60.0))

		self.is_annotating = False
		self.commands = GraspingHelper.get_available_commands()
		self.commands["save"] = self.write_grasps

	def object_callback(self, msg):
		self.objects = []
		self.object_poses = dict()
		for object in msg.objects:
			self.objects.append(object.type.key)
			self.object_poses[object.type.key] = GraspingHelper.getPoseStampedFromPoseWithCovariance(object.pose)
		self.broadcast_transforms()

		if not self.is_annotating:
			self.is_annotating = True
			self.current_thread = Thread(None, self.annotate_grasps)
			self.current_thread.start()

	def broadcast_transforms(self):
		for object, pose in self.object_poses.iteritems():
			origin = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
			orientation = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
			self.broadcaster.sendTransform(origin, orientation, pose.header.stamp, str(object), pose.header.frame_id)

	def annotate_grasps(self):
		object_id = GraspingHelper.get_name(self.objects)
		gripper = GraspingHelper.get_gripper()
		frame_id = "/reference/" + gripper + "_gripper"
		print("Frame id: " + frame_id)
		self.grasps = []
		keep_going = True
                print "saving time"
                time = rospy.Time.now()
		index = 0
		while keep_going:
			response = "continue"
			while (len(response) > 0 and response not in self.commands.keys()):
				response = raw_input("Press enter to annotate the grasp or type 'save' to end annotation. ")

			grasps = self.commands[response](self.transformer, gripper, frame_id, str(object_id), index, time)
			if response == "save":
				return
			index += 1
			self.grasps.extend(grasps)
			try:
				to_publish_grasps = MoveHelper.set_grasps_at_pose(self.object_poses[object_id], self.grasps, self.transformer)
				self.publish_grasp_markers(to_publish_grasps, object_id)
			except KeyError as e:
				pass
			
	
	def publish_grasp_markers(self, grasps, object_id):
		#for grasp in grasps:
		#	print(str(grasp.grasp_pose))
		markers = MoveHelper.create_grasp_markers(grasps, object_id)
		for marker in markers:
			#print(str(marker))
			self.markers_publisher.publish(marker)
		
	def write_grasps(self, *args):
		GraspingHelper.write_grasps(self.grasps)

	def go(self):
		rospy.spin()


if __name__=='__main__':
	rospy.init_node("annonate_grasps")
	annotator = Annotator()
	annotator.go()
