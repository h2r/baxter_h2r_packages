#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_grasps_server")

import rospy
import yaml
import sys
import genpy
import tf
import copy
import math
from tf import transformations
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import Grasp


def usage():
	print("python cylinder_grasp_generator.py <start_grasp_file.yaml> <output filename>")

def load_grasps(filename):
		f = open(filename)
		args = yaml.load(f)
		grasp = Grasp()
		genpy.message.fill_message_args(grasp, args)
		return grasp

def generate_grasps(base_grasp, number = 36):
	roll, pitch, yaw = get_array_from_quaternion(base_grasp.grasp_pose.pose.orientation)
	grasps = []
	for i in range(number):
		newYaw = yaw + (float(i) / number) * 2 * math.pi
		newGrasp = copy.deepcopy(base_grasp)
		newQuat = tf.transformations.quaternion_from_euler(roll, pitch, newYaw)
		dX = -0.21 * math.cos(newYaw)
		dY = -0.21 * math.sin(newYaw)
		newGrasp.grasp_pose.pose.orientation = get_quaternion_from_array(newQuat)
		newGrasp.grasp_pose.pose.position.x += dX
		newGrasp.grasp_pose.pose.position.y += dY
		newGrasp.id = "'" + str(i) + "'"
		grasps.append(newGrasp)
	return grasps

def get_array_from_quaternion(quaternion):
	arry = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
	return tf.transformations.euler_from_quaternion(arry)

def get_quaternion_from_array(arry):
	return Quaternion(x=arry[0], y=arry[1], z=arry[2], w=arry[3])

def writeGrasps(grasps, filename):
	stream = file(filename, 'w')
	args = []
	for grasp in grasps:
		str = genpy.message.strify_message(grasp)
		args.append(yaml.load(str))

	yaml.dump(args, stream)
	print(yaml.dump(args))

if __name__ == '__main__':
	argv=sys.argv
	argv = rospy.myargv(argv)
	if (len(argv)==1):
		usage()
	filename = argv[1]
	output = argv[2]
	base_grasp = load_grasps(filename)
	grasps = generate_grasps(base_grasp, 8)
	writeGrasps(grasps, output)