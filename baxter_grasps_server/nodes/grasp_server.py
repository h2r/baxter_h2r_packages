#!/usr/bin/env python

import genpy
import yaml
import sys
import rospy
import os
import actionlib

from std_msgs.msg import String
from moveit_msgs.msg import Grasp
from grasp_server.msg import GraspServiceRequest, GraspServiceResponse

class grasp_server:
	
	def go(self, grasp_dir):
		self.grasps = dict()
		files = self.get_files(grasp_dir)
		for name, filename in files.iteritems():
			self.grasps[name] = self.load_grasps(filename)
		rospy.Service('/grasp_server', GraspServiceRequest, grasp_callback)
		rospy.spin()
	
	def get_files(self, grasp_dir):
		file_paths = dict()
		for root, directories, files in os.walk(grasp_dir):
			for filename in files:
				filepath = os.path.join(root, filename)
				obj = os.path.basename(root)
				file_paths[obj] = filepath
		return file_paths
	
	def load_grasps(self, filename):
		f = open(filename)
		args = yaml.load(f)
		grasp = Grasp()
		genpy.message.fill_message_args(grasp, args)
		return grasp

	def grasp_callback(self, request):
		if (request.name in self.grasps.keys()):
			return GraspServiceResponse(success=true, grasps=self.grasps[request.name])
		return GraspServiceResponse(success=false)

def usage():
	print("""
	\trosrun baxter_grasps_server grasp_server <grasps directory>
""")
	sys.exit(getattr(os, 'EX_USAGE', 1))

if __name__ == '__main__':
	rospy.init_node("grasp_server")
	argv=sys.argv
	argv = rospy.myargv(argv)
	if (len(argv)==1):
		usage()
	
	directory = argv[1]

	server = grasp_server()
	server.go(directory)
	
