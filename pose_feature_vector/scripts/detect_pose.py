#!/usr/bin/env python


import sys
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped,  Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler
from pose_feature_vector.msg import PersonFVMsg
from std_msgs.msg import Header, String
from baxter_props import BaxterProps

from sklearn import linear_model


global logreg
global classToPoseMap
global props
global behaviorOccurring
global curRecognizedPose
global numConsecutiveInPose

behaviorOccurring = False
curRecognizedPose = 3
numConsecutiveInPose = 0

classToPoseMap = {0: 'hug', 1: 'high five', 2: 'fist bump', 3: 'null pose', 4: 'psi'}

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
	global props
	global behaviorOccurring
	global curRecognizedPose
	global numConsecutiveInPose


	if not behaviorOccurring:

		behaviorOccurring = True

		fv = vecSansDistance(data)

		classId = int(logreg.predict(fv)[0])
		className = classToPoseMap[classId]

		if classId != curRecognizedPose:
			curRecognizedPose = classId
			numConsecutiveInPose = 0

		elif curRecognizedPose != 3 and curRecognizedPose != 4:

			if numConsecutiveInPose < 15:
				numConsecutiveInPose += 1
				print 'num consecutive', className, numConsecutiveInPose

			else:

				print 'performing:', className
				numConsecutiveInPose = 0

				
				try:

					if classId == 0:
						props.hug(0.4, 10.0)
					elif classId == 1:
						props.five(Point(x=0.56, y=0.81, z=0.86), "left")
					elif classId == 2:
						props.bump(Point(x=0.9, y=0.7, z=0.25), "left")

				except:
					print 'Could not process request'
				
				

		behaviorOccurring = False

	else:
		print 'skipping'



def main():

	global logreg
	global props

	print 'loading classifier'
	data = np.genfromtxt(sys.argv[1], delimiter=',')
	X = data[:, 1:]
	Y = data[:, 0]

	logreg = linear_model.LogisticRegression(C=1e5)
	logreg.fit(X, Y)

	print 'ready for inputs'

	

	rospy.init_node('pose_detector')

	props = BaxterProps()

	rospy.Subscriber('pose_feature_vector', PersonFVMsg, callback, queue_size=1)

	rospy.spin()


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print "usage: detect_pose.py <training data file>"
		sys.exit()
	main()