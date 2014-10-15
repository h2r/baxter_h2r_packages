#!/usr/bin/env python

import roslib
roslib.load_manifest("object_identifier")
import rospy
import copy

from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseWithCovarianceStamped

defined_objects = [
	{"class": "gyrobowl", "name": "cocoa_bowl"},# 0
	{"class": "gyrobowl", "name": "flour_bowl"},# 1
	{"class": "gyrobowl", "name": "salt_bowl"},# 2
	{"class": "gyrobowl", "name": "baking_powder_bovl"},# 3
	{"class": "gyrobowl", "name": "eggs_bowl"},# 4
	{"class": "gyrobowl", "name": "white_sugar_bowl"},# 5
	{"class": "gyrobowl", "name": "vanilla_bowl"},# 6
	{"class": "gyrobowl", "name": "butter_bowl"},# 7
	{"class": "mixing_bowl", "name": "mixing_bowl_1"}, # 8
	{"class": "mixing_bowl", "name": "mixing_bowl_2"}, # 9
	{"class": "spoon", "name": "spoon"},# 10
	{"class": "spoon", "name": "whisk"},# 11
]


class ObjectIdentifier:
	def __init__(self):
		#self.ork_objects = []
		#self.joberlin_objects = []
		#rospy.Subscriber("/recognized_object_array", self.ork_callback)
		#rospy.Subscriber("/joberlin_detection", self.joberlin_callback)
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_marker_callback)
		self.ar_object_publisher = rospy.Publisher("/ar_objects", RecognizedObjectArray)

	def ork_callback(self, msg):
		self.ork_objects = msg.objects

	def joberlin_callback(self, msg):
		self.joberlin_objects = msg.objects

	def ar_marker_callback(self, msg):
		objects_msg = RecognizedObjectArray()

		objects_msg.header = copy.deepcopy(msg.header)
		for marker in msg.markers:
			if marker.id < len(defined_objects) and marker.pose.pose.position.z > 0.3:
				object = RecognizedObject()
				
				object.pose = self.get_pose_from_pose(marker.pose)
				#object.pose.pose.pose.orientation.w = 1.0
				object.pose.header = copy.deepcopy(marker.header)
				#object.pose.header.frame_id = "ar_marker_" + str(marker.id)
				object.type.key = defined_objects[marker.id]["name"]
				objects_msg.objects.append(object)
		self.ar_object_publisher.publish(objects_msg)

	def get_pose_from_pose(self, pose_stamped):
		pose = PoseWithCovarianceStamped()
		pose.header = pose_stamped.header
		pose.pose.pose = pose_stamped.pose
		return pose

if __name__ == "__main__":
	rospy.init_node("object_identifier")
	identifier = ObjectIdentifier()
	rospy.spin()