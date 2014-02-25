#!/usr/bin/env python
import subprocess
import rospy
import roslib
from std_msgs.msg import String
import os

try:
	pub = rospy.Publisher('speech_command', String)
	rospy.init_node('speaker')
	
	jar_filename = roslib.packages.get_pkg_dir("baxter_speech_command") + "/src/bin/BaxterSpeech.jar"

	proc = subprocess.Popen(["java","-mx312m", "-jar", jar_filename], stdout=subprocess.PIPE)
#	proc = subprocess.Popen(["java","-mx312m", "-jar", "../src/bin/BaxterSpeech.jar"], stdout=subprocess.PIPE)
	print "Start Speaking. Press Crtl-C to quit."
	while True:
		line = proc.stdout.readline()
		if line != '':
			if len(line) != 1:
				rospy.loginfo(line.rstrip())
				pub.publish(String(line.rstrip()))
				rospy.sleep(1.0)
		else:
			break
except rospy.ROSInterruptException:
	pass
