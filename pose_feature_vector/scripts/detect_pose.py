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
from baxter_props import BaxterProps

from sklearn import linear_model
import rospkg
from subprocess import call


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

			if numConsecutiveInPose < 2:
				#TODO NAKUL publish "Detected <className> image, please hold pose"
				if classId == 0:
					rospy.loginfo("Robot hugging prep")
					#TODO NAKUL publish "executing hug image"
					imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
					imageStr="--file="+imagePath+"/images/hugPrep.png"
					call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
					
				elif classId == 1:
					rospy.loginfo("Robot high fiving prep")
					#TODO NAKUL publish "executing high five image"
					imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
					imageStr="--file="+imagePath+"/images/highFivePrep.png"
					call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
					
				elif classId == 2:
					rospy.loginfo("Robot fist bumping prep")
					#TODO NAKUL publish "executing fist bump image"
					imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
					imageStr="--file="+imagePath+"/images/fistBumpPrep.png"
					call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])

				numConsecutiveInPose += 1
				rospy.loginfo("Detected " + className + ". Please hold pose")

			else:

				print 'performing:', className
				numConsecutiveInPose +=1
				if classId == 0:
					rospy.loginfo("Robot is hugging")
					#TODO NAKUL publish "executing hug image"
					imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
					imageStr="--file="+imagePath+"/images/hug.png"
					call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
					props.hug(0.4, 10.0)
				elif classId == 1:
					rospy.loginfo("Robot is high fiving")
					#TODO NAKUL publish "executing high five image"
					imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
					imageStr="--file="+imagePath+"/images/highFive.png"
					call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
					props.five(Point(x=0.56, y=0.81, z=0.86), "left")
				elif classId == 2:
					rospy.loginfo("Robot is fist bumping")
					#TODO NAKUL publish "executing fist bump image"
					imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
					imageStr="--file="+imagePath+"/images/fistBump.png"
					call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
					props.bump(Point(x=0.9, y=0.7, z=0.25), "left")
			
				
		else:
			rospy.loginfo("Please pose with either hug/high five/fist bump")
			imagePath=roslib.packages.get_pkg_dir('pose_feature_vector')
			imageStr="--file="+imagePath+"/images/prepAll.png"
			call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
			#TODO NAKUL publish "Pose suggestion image"
			pass

		behaviorOccurring = False

	else:
		pass



def main():

	global logreg
	global props

	rospy.loginfo("Loading classifier")
	data = np.genfromtxt(sys.argv[1], delimiter=',')
	X = data[:, 1:]
	Y = data[:, 0]

	logreg = linear_model.LogisticRegression(C=1e5)
	logreg.fit(X, Y)

	rospy.loginfo("Ready for inputs")


	rospy.init_node('pose_detector')

	props = BaxterProps()

	rospy.Subscriber('pose_feature_vector', PersonFVMsg, callback, queue_size=1)

	rospy.spin()


if __name__ == '__main__':
	if len(sys.argv) < 2:
		print "usage: detect_pose.py <training data file>"
		sys.exit()
	main()
