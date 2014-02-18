#!/usr/bin/env python
"""
This file is a client to a server: it keeps making request and prints the output
from the ORK actionlib server
"""
import actionlib
import argparse
import rospy
import sys
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

#resultMessage

def commandClient():
	client = actionlib.SimpleActionClient('do_all_client', ObjectRecognitionAction)
	client.wait_for_server()
	goal = ObjectRecognitionGoal()
	client.send_goal(goal, done_cb=on_result)
	#client.wait_for_result()


def callback(data):
	rospy.loginfo(rospy.get_name() + ": I heard %f" % data.scale.x)

def topicLister():
	print "here"
	#rospy.Subscriber("/tabletop/clusters", MarkerArray, callback)
	print "there"

def on_result(status, result):
    print result
    print type(result)
    #resultMessage = result

if __name__ == '__main__':
    
    #SB: You should name the node something meaningful
    rospy.init_node('do_all_client')

    #SB: name rename callback to something more desciptive (i.e. clustersCB, clustersCallback etc.)
    rospy.Subscriber("/tabletop/clusters", MarkerArray, callback)

    #SB: When creating a client, you should put in the action name, not the current node name
    #SB: What client is this? Can you make the var name more descriptive
    client = actionlib.SimpleActionClient('do_all_client', ObjectRecognitionAction)
    client.wait_for_server()

    #SB: Why are you subscribing to clusters twice?
    rospy.Subscriber("/tabletop/clusters", MarkerArray, callback)

    #SB: You need an additional subscriber to a string topic (so we can send object requests)

    #SB: This code should not be in main, it should be in the string topic callback
    goal = ObjectRecognitionGoal()


    client.send_goal(goal, done_cb=on_result)
    #commandClient()
    #topicLister()
    rospy.spin()
    '''
    #client = actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
    client.wait_for_server()

    start = rospy.Time.now()  # for checking the round trip time.

    goal = ObjectRecognitionGoal()

    # Sample region of interest for object detection (disabled by default)
    # goal.use_roi = True
    # goal.filter_limits = [-0.4, 0.4, -1.0, 0.2, 0.01, 1.5]

    client.send_goal(goal, done_cb=on_result)
    client.wait_for_result()  # wait indefinitely for a result

    # print out the round trip time.
    print "Time for 1 detection:", (rospy.Time.now() - start).to_sec()
    # take filtered points, find centroid of the object (x,y) and centroid of the 50 - 100 (?) for (z) highest points to grasp at that centroid 
    #
    '''
    
    

