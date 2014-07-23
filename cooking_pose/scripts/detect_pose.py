#!/usr/bin/env python


import sys
import roslib
roslib.load_manifest("pose_feature_vector")
import rospy

import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped,  Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler
from pose_feature_vector.msg import PersonFVMsg
from std_msgs.msg import Header, String

from sklearn import linear_model
import rospkg
from subprocess import call

global pub
global logreg
global classToPoseMap
global transition_prob
transition_prob = 0.9
global curr_estimates
curr_estimates = dict()
global prev_estimates
prev_estimates = dict()
global classToPoseMap
classToPoseMap = {0: 'null pose', 1: 'stirring', 2: 'pouring', 3: 'mixing', 4: 'moving'}

def vec3ToArray(v3):
	return [v3.x, v3.y, v3.z]

def vecSansDistance(msg):
	fv = []
	fv += vec3ToArray(msg.rh)
	fv += vec3ToArray(msg.re)
	fv += vec3ToArray(msg.lh)
	fv += vec3ToArray(msg.le)
	return fv



def callback(data):

	global classToPoseMap
	global logreg
	global curr_estimates
	global prev_estimates


	fv = vecSansDistance(data)
	probabilities = logreg.predict_proba(fv)
	for i in classToPoseMap.keys():
		prob_sum = 0.0
		for j in classToPoseMap.keys():
			if i == j:
				prob_sum += transition_prob * prev_estimates[j]
			else:
				prob_sum += (1-transition_prob) * prev_estimates[j]
		curr_estimates[i] = prob_sum * probabilities[0][i]
	total_prob = sum(curr_estimates.values(), 0.0)
	for key in curr_estimates.keys():
		curr_estimates[key] = curr_estimates[key]/total_prob
	classid = int(max(curr_estimates, key=lambda x: curr_estimates[x]))
	className = classToPoseMap[classId]
	pub.publish(className)
	prev_estimates = curr_estimates
	curr_estimates = dict()


def main():

	global logreg
	global prev_estimates
	rospy.loginfo("Loading classifier")
	data = np.genfromtxt(sys.argv[1], delimiter=',')
	X = data[:, 1:]
	Y = data[:, 0]

	logreg = linear_model.LogisticRegression(C=1e5)
	logreg.fit(X, Y)
	keys = classToPoseMap.keys()
	for i in keys:
		prev_estimates[i] = 1.0/len(keys)

	rospy.loginfo("Ready for inputs")


	rospy.init_node('pose_detector')
	pub = rospy.Publisher("cooking_pose", String, queue_size=1)

	rospy.Subscriber('pose_feature_vector', PersonFVMsg, callback, queue_size=1)

	rospy.spin()


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print "usage: detect_pose.py <training data file>"
		sys.exit()
	main()
