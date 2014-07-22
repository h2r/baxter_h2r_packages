#!/usr/bin/env python

import sys
import rospy
import tf
import math
import threading
from geometry_msgs.msg import PoseStamped,  Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler
from pose_feature_vector.msg import PersonFVMsg
from std_msgs.msg import Header, String

global poseID
global count

def vec3ToArray(v3):
	return [v3.x, v3.y, v3.z]

def vecSansDistance(msg):
	fv = []
	fv += vec3ToArray(msg.rh)
	fv += vec3ToArray(msg.re)
	fv += vec3ToArray(msg.lh)
	fv += vec3ToArray(msg.le)
	return fv


def timerCallback(strRep):
	f = open(sys.argv[1], 'a')
	f.write(strRep)
	f.close()
	print strRep

def callback(data):

	global poseID
	global count

	fv = vecSansDistance(data)
	strRep = poseID + ',' + ','.join([str(e) for e in fv]) + '\n'
	t = threading.Timer(4, timerCallback, args=[strRep])
	t.start()
	print count
	count += 1
	


def main():

	global poseID
	global count
	poseID = sys.argv[2]


	count = 0

	rospy.init_node('pose_fv_recorder')
	rospy.Subscriber('pose_feature_vector', PersonFVMsg, callback)

	rospy.spin()


if __name__ == '__main__':
	if len(sys.argv) != 3:
		print 'required format:\n\tpathToFile poseID'
		sys.exit()
	main()