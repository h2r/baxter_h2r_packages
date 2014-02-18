#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
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
    
    rospy.init_node('do_all_client')
    rospy.Subscriber("/tabletop/clusters", MarkerArray, callback)
    client = actionlib.SimpleActionClient('do_all_client', ObjectRecognitionAction)
    client.wait_for_server()
    rospy.Subscriber("/tabletop/clusters", MarkerArray, callback)
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
    
    

