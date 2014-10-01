#!/usr/bin/env python
import roslib
roslib.load_manifest("object_identifier")
import rospy
import tf
import tf.transformations
from object_recognition_msgs.msg import TableArray, Table
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
from baxter_pick_and_place.move_helper import MoveHelper

class TableTransformer:
	def __init__(self):
		self.transformer = tf.TransformListener()
		rospy.Subscriber("table_array", TableArray, self.table_callback)
		self.publisher = rospy.Publisher("table_array_transformed", TableArray)
		self.is_finished = True

	def table_callback(self, msg):
		try:
			table_array = TableArray()
			table_array.header.frame_id = "world"
			highest_table = None
			for table in msg.tables:
				new_table = Table()
				new_table.header.frame_id = "world"

				table_pose = PoseStamped(pose=table.pose, header = table.header)
				table_pose.header.stamp = self.transformer.getLatestCommonTime("world", table.header.frame_id)
				new_table.pose = self.transformer.transformPose("world", table_pose).pose

				new_table.convex_hull = table.convex_hull
				table_array.tables.append(new_table)

				if highest_table is None or new_table.pose.position.z > highest_table.pose.position.z:
					highest_table = new_table

			self.publisher.publish(table_array)
			position = highest_table.pose.position
			#position.z -= 0.1
			MoveHelper.add_table(position=position)
		#	rospy.sleep(10)
			self.is_finished = True
		except:
			self.is_finished = False
		##if self.is_finished:
		#	rospy.signal_shutdown("Table has been added, table transformer is exiting")



if __name__ == "__main__":
	global transformer, publisher
	rospy.init_node("table_transformer")
	transformer = TableTransformer()
	
	rospy.spin()