#!/usr/bin/env python

import sys
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped,  Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler
from pose_feature_vector.msg import PersonFVMsg
from std_msgs.msg import Header, String
import rospkg
from subprocess import call

global tfListener
global frameID
global seq
global userID


def mag3(vec):
	return math.sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])

def norm3(vec):
	m = mag3(vec)
	return (vec[0]/m, vec[1]/m, vec[2]/m)

def translate3(point, origin):
	return (point[0]-origin[0], point[1]-origin[1], point[2]-origin[2])

def getFeatureVector(now):

	global tfListener
	global userID

	trh = tfListener.lookupTransform(frameID, "right_hand_" + userID, now)
	tlh = tfListener.lookupTransform(frameID, "left_hand_" + userID, now)
	tre = tfListener.lookupTransform(frameID, "right_elbow_" + userID, now)
	tle = tfListener.lookupTransform(frameID, "left_elbow_" + userID, now)
	thead = tfListener.lookupTransform(frameID, "head_" + userID, now)
	
	trrh = norm3(translate3(trh[0], thead[0]))
	trlh = norm3(translate3(tlh[0], thead[0]))
	trre = norm3(translate3(tre[0], thead[0]))
	trle = norm3(translate3(tle[0], thead[0]))

	fv = []
	fv += trrh
	fv += trlh
	fv += trre
	fv += trle
	fv.append(thead[0][0]) #head distance

	return fv


def setV3(v3, fv, sIndex):
	v3.x = fv[sIndex]
	v3.y = fv[sIndex+1]
	v3.z = fv[sIndex+2]

def fvToMsg(fv, now):

	global seq

	fvMsg = PersonFVMsg()
	
	header = Header()
	header.seq = seq
	header.stamp = now
	header.frame_id = frameID

	rhv = Vector3()
	rev = Vector3()
	lhv = Vector3()
	lev = Vector3()

	setV3(rhv, fv, 0)
	setV3(lhv, fv, 3)
	setV3(rev, fv, 6)
	setV3(lev, fv, 9)

	fvMsg.header = header
	fvMsg.rh = rhv
	fvMsg.re = rev
	fvMsg.lh = lhv
	fvMsg.le = lev

	fvMsg.distance = fv[12]

	return fvMsg

def setMostRecentUser():
	global tfListener
	global frameID
	global userID

	allFramesString = tfListener.getFrameStrings()
	onlyUsers = set([line for line in allFramesString if 'right_elbow_' in line])
	n = len('right_elbow_')
	userIDs = [el[n:] for el in onlyUsers]
	if len(userIDs) > 0:
		mostRecentUID = userIDs[0]
		mostRecentTime = tfListener.getLatestCommonTime(frameID, 'right_elbow_' + mostRecentUID).secs
		for uid in userIDs:
			compTime = tfListener.getLatestCommonTime(frameID, 'right_elbow_' + uid).secs
			if compTime > mostRecentTime:
				mostRecentUID = uid
				mostRecentTime = compTime
		userID = mostRecentUID
	else:
		userID = ''
			





def main():

	global seq
	global tfListener
	global userID
	seq = 0

	pub = rospy.Publisher('pose_feature_vector', PersonFVMsg)
	rospy.init_node('poseFV')

	tfListener = tf.TransformListener()

	r = rospy.Rate(10) #1hz

	publishWait = 0

	#filePath = sys.argv[1]

	print 'waiting for user'
	while not rospy.is_shutdown():
		now = rospy.Time(0)

		setMostRecentUser()
		
		if userID != '':
			try:
				tfListener.waitForTransform(frameID, "right_hand_" + userID, now, rospy.Duration(1))
				if publishWait == 0:
					fv = getFeatureVector(now)
					#print fv
					curMsg = fvToMsg(fv, now)
					pub.publish(curMsg)
					seq += 1

				else:
					print 'publishing in', publishWait
					publishWait -= 1
			except tf.Exception:
				print "waiting for user"
		else:
			pass
			#TODO NAKUL publish "Waiting for user, calibration how to image"
			imagePath=rospack.get_path('pose_feature_vector')
			imageStr="--file="+imagePath+"/images/psi.png"
			call(["rosrun", "baxter_examples", "xdisplay_image.py", imageStr])
			rospy.loginfo("Waiting for user")

		r.sleep()
		

if __name__ == '__main__':
	#note that you should set: rosparam set openni_tracker/camera_frame_id "camera_depth_optical_frame"
	#before tracker code is launched
	frameID = str(rospy.get_param('/openni_tracker/camera_frame_id'))
	#frameID = 'openni_depth_frame'
	print frameID
	try:
		main()
	except rospy.ROSInterruptException: pass
