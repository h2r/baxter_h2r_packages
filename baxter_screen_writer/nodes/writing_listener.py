#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_screen_writer")
import rospy

from std_msgs.msg import String
from baxter_screen_writer.screen_writer import ScreenWriter

	


if __name__ == "__main__":
	def cb(msg):
		writer.write(msg.data)

	rospy.init_node("screen_writer")
	writer = ScreenWriter()

	rospy.Subscriber("/robot/screen/text", String, cb)
	rospy.spin()	
#	rate = rospy.Rate(30)
#	index = 2
#	writer.write("Hello")
#	while not rospy.is_shutdown():
#		writer.write(str(index))
#		index *= 2
#		rate.sleep()