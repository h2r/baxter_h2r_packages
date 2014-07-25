#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_screen_writer")
import rospy


from baxter_screen_writer.screen_writer import ScreenWriter

if __name__ == "__main__":
	rospy.init_node("screen_writer")
	writer = ScreenWriter()
	rate = rospy.Rate(30)
	index = 2
	while not rospy.is_shutdown():
		writer.write(str(index))
		index *= 2
		rate.sleep()