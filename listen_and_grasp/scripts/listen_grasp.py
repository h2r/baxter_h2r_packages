#!/usr/bin/env python
"""
This file is a client to a server: it keeps making request and prints the output
from the ORK actionlib server
"""
import actionlib
import argparse
import rospy
import sys
import tf
#import tf2
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import numpy as np

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp

global control_arm
#resultMessage
global client
global pointRobotFrame
pointRobotFrame = []

def getMedian(numericValues):
  theValues = sorted(numericValues)

  if len(theValues) % 2 == 1:
    return theValues[(len(theValues)+1)/2-1]
  else:
    lower = theValues[len(theValues)/2-1]
    upper = theValues[len(theValues)/2]

    return (float(lower + upper)) / 2  

def speechCallback(data):
	rospy.loginfo(rospy.get_name() + ": I heard %s" % str(data))
	#start = rospy.Time.now()  # for checking the round trip time.
	goal = ObjectRecognitionGoal()
	# Sample region of interest for object detection (disabled by default)
	# goal.use_roi = True
	goal.filter_limits = [-2, 2, -2.0, 2.0, 0.01, 1.5]
	client.send_goal(goal, done_cb=on_result)

def pointsCallback(data):
	#rospy.loginfo(str(len(data.markers)))
	#rospy.loginfo(str(len(data.markers[1].points)))
	#rospy.loginfo(str(len(data.markers[0].points)))
	#pointNew = data.markers
	#header =
	var= tf.TransformListener()
	#var= tf.Transformer(True, rospy.Duration(10.0))
	pointCameraFrame = []
	now=rospy.Time(0)
	
	tempPoint = PointStamped()
	tempPoint.header = data.markers[0].header
	tempPoint.point = data.markers[0].points[0]
	var.waitForTransform("/world", tempPoint.header.frame_id, rospy.Time(0), rospy.Duration(1))
	tempPoint.header.stamp=rospy.Time(0)
	transformMat=var.asMatrix("/world", tempPoint.header)
	str(transformMat)
	maxPoint=0.0;
	maxMarkerZList=[]
	maxMarkerindexList=[]
	for x in range(len(data.markers)):
		tempMax=0.0
		tempIndex=0
		for y in range(len(data.markers[x].points)):
			cameraPoint = np.asmatrix(np.array([[data.markers[x].points[y].x,data.markers[x].points[y].y,data.markers[x].points[y].z,1.0]]))
			#robotPoint = transformMat* np.asmatrix(cameraPoint)
			#print robotPoint.shape
			#print cameraPoint.shape
			#print transformMat.shape
			robotPoint = transformMat* cameraPoint.transpose()
			#print robotPoint.shape			
			if robotPoint[2] > tempMax:
				tempMax=robotPoint[2]
				tempIndex=y
		maxMarkerZList.append(tempMax)
		maxMarkerindexList.append(tempIndex)
	Max=0.0
	idx=0
	for x in range(len(maxMarkerZList)):
		if maxMarkerZList[x] > Max:
			Max=maxMarkerZList[x]
			idx=x
	sum_x=0.0
	sum_y=0.0	
	for clusterPoint in range(len(data.markers[idx].points)):
		sum_x=sum_x+data.markers[idx].points[clusterPoint].x
		sum_y=sum_y+data.markers[idx].points[clusterPoint].y
	pointx = sum_x/len(data.markers[idx].points)
	pointy = sum_y/len(data.markers[idx].points)
	'''
	arrayX=[]
	arrayY=[]
	for clusterPoint in range(len(data.markers[idx].points)):
		sum_x=sum_x+data.markers[idx].points[clusterPoint].x
		sum_y=sum_y+data.markers[idx].points[clusterPoint].y
	'''
	#grabPointCamera= np.asmatrix(np.array([[data.markers[idx].points[maxMarkerindexList[idx]].x,data.markers[idx].points[maxMarkerindexList[idx]].y,data.markers[idx].points[maxMarkerindexList[idx]].z,1.0]]))
	grabPointCamera= np.asmatrix(np.array([[pointx,pointy,data.markers[idx].points[maxMarkerindexList[idx]].z,1.0]]))
	grabPoint = transformMat *grabPointCamera.transpose()
	#np.array([data.markers[idx].points[maxMarkerindexList[idx]].x,data.markers[idx].points[maxMarkerindexList[idx]].y,data.markers[idx].points[maxMarkerindexList[idx]].z,1.0])
	print str(grabPoint)
	moveit_commander.roscpp_initialize(sys.argv)
	#rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	scene.remove_world_object("pole")
	scene.remove_world_object("table")
	scene.remove_world_object("part")
	scene.remove_world_object("cube")
	group = moveit_commander.MoveGroupCommander("left_arm")
	group.set_start_state_to_current_state()
	left = baxter_interface.Gripper('left')
	left.calibrate()
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
	left.open()
	print str(grabPoint)
	rospy.sleep(1)
	
        control_arm.move_to_neutral()
        

	p = PoseStamped()
	p.header.frame_id = "/base"
	p.pose.position.x = float(grabPoint[0])
	p.pose.position.y = float(grabPoint[1])
	p.pose.position.z = float(grabPoint[2])
	#scene.add_box("cube", p, (0.05, 0.05, 0.05))
	
	p.pose.position.x = 0.5
	p.pose.position.y = 0.5
	p.pose.position.z = -0.25
	scene.add_box("table", p, (0.5, 1.5, 0.2))
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = float(grabPoint[0]) - 0.11
	pose_target.pose.position.y = float(grabPoint[1]) - 0.01
	pose_target.pose.position.z = float(grabPoint[2]) + 0.15
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 1"
	rospy.sleep(5)
	group.go()
	print "\n here now 2"
	rospy.sleep(5)
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = float(grabPoint[0]) - 0.11
	pose_target.pose.position.y = float(grabPoint[1]) - 0.01
	pose_target.pose.position.z = float(grabPoint[2]) - 0.02
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 3"
	rospy.sleep(2)
	group.go()
	left.close()
	#group.attach_object("cube")
	rospy.sleep(2)
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = float(grabPoint[0]) - 0.11
	pose_target.pose.position.y = float(grabPoint[1]) - 0.01
	pose_target.pose.position.z = float(grabPoint[2]) + 0.15
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 4"
	rospy.sleep(2)
	group.go()
	rospy.sleep(2)
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = float(grabPoint[0]) + 0.05
	pose_target.pose.position.y = float(grabPoint[1]) + 0.1
	pose_target.pose.position.z = float(grabPoint[2]) + 0.15
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 5"
	rospy.sleep(5)
	group.go()
	
	rospy.sleep(2)
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = float(grabPoint[0]) + 0.05
	pose_target.pose.position.y = float(grabPoint[1]) + 0.1
	pose_target.pose.position.z = float(grabPoint[2]) - 0.02
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 5 - extra safe"
	rospy.sleep(5)
	group.go()
	left.open()
	#group.detach_object("cube")
	
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = float(grabPoint[0]) + 0.05
	pose_target.pose.position.y = float(grabPoint[1]) + 0.1
	pose_target.pose.position.z = float(grabPoint[2]) + 0.15
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 6"
	rospy.sleep(5)
	group.go()
	
	pose_target = geometry_msgs.msg.PoseStamped()
	pose_target.header.frame_id = "/base"
	pose_target.pose.orientation.x = 0.
	pose_target.pose.orientation.y = 0.707
	pose_target.pose.orientation.z = 0
	pose_target.pose.orientation.w = 0.707
	pose_target.pose.position.x = 0.8
	pose_target.pose.position.y = 0.5
	pose_target.pose.position.z = 0.5
	group.set_pose_target(pose_target)
	group.plan()
	print "\n here now 7"
	rospy.sleep(5)
	group.go()
	
	
	
	# =============== QUITTING ================
	#quit()
			#tempPoint = PointStamped()
			#tempPoint.header = x.header
			#tempPoint.point = y
			#print str(tempPoint)
			#var.waitForTransform("/world", tempPoint.header.frame_id, rospy.Time(0), rospy.Duration(1))
			#var.lookupTransform("/world", tempPoint.header.frame_id, now)
			#tempPointRobot=var.transformPoint("/world",tempPoint)
			#tempPoint.header.stamp=rospy.Time(0)
			#print str(var.asMatrix("/world", tempPoint.header))
						
			#pointRobotFrame.append(tempPointRobot)
	#rospy.loginfo(str(len(pointRobotFrame)))
	

        '''#method 1
        transformListener.transformPoint(targetFrame, sourceFrame, pointIn, pointOut)
e
        #method 2
        transformListener.lookupTransform(targetFrame, sourceFrame, ROS.now(), transform)
        pointOut = transform * pointIn
 
	#for()
	'''
	#pass

def on_result(status, result):
    print result

if __name__ == '__main__':
    rospy.init_node('do_all_client')
    control_arm = baxter_interface.limb.Limb("left")
    client = actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
    client.wait_for_server()
    rospy.Subscriber("/tabletop/clusters", MarkerArray, pointsCallback)
    rospy.Subscriber("/text", String, speechCallback)
    rospy.spin()

