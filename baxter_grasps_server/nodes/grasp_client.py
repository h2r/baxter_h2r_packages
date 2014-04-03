#!/usr/bin/env python

import genpy
import yaml
import sys
import rospy
import os
import actionlib

from std_msgs.msg import String
from moveit_msgs.msg import Grasp
from baxter_grasps_server.srv import GraspService

if __name__ == '__main__':
	rospy.init_node("grasp_client")
	rospy.wait_for_service('/grasp_server')
	try:
		grasp_server = rospy.ServiceProxy('/grasp_server', GraspService)
		grasps = grasp_server("coconut")
		print(str(grasps))
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e