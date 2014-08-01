#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_screen_writer")
import rospy

import Image
import ImageDraw
import ImageFont
import textwrap
import math

from threading import Thread
from sensor_msgs.msg import Image as ROSImage

class ScreenWriter:

	def __init__(self):
		self.image_publisher = rospy.Publisher("/robot/xdisplay", ROSImage)
		self.font = roslib.packages.get_pkg_dir("baxter_screen_writer") + "/fonts/arial.ttf"
		self.write("")

	def write(self, text):

		lines = self.wrap(text)
		longest_line = self.longest(lines)

		pil_image = Image.new("RGB"	, (1024, 600), (0, 0, 0))
		d = ImageDraw.Draw(pil_image)


		fontsize = self.get_fontsize(d, longest_line, 100)
		font = ImageFont.truetype(self.font, fontsize)

		width, height = self.text_size(d, longest_line, font)
		text_height = height * len(lines)

		
		for i in range(len(lines)):
			position = self.get_position(width, text_height, height, i)
			d.text(position, lines[i], fill=(0, 255, 0), font=font)
		self._image = self.pil_to_imgmsg(pil_image)
		self.publish_image()

	def wrap(self, text):
		min_width = 16
		if len(text) < min_width:
			return [text]
		num_chars = len(text)
		width = math.sqrt(num_chars * 5)
		width = max(int(width), min_width)
		return textwrap.wrap(text, width=width)

	def longest(self, lines):
		max_length = 0
		index = 0
		for i in range(len(lines)):
			if len(lines[i]) > max_length:
				max_length = len(lines[i])
				index = i
		return lines[index]

	def text_size(self, image_draw, text, font):
		return image_draw.textsize(text, font)

	def get_position(self, width, text_height, height, index):
		start_y = (600.0 - text_height) / 2.0
		y = start_y + index * height
		return ((1024.0 - width) / 2.0, y)

	def get_fontsize(self, image_draw, text, fontsize):
		if text == "":
			return fontsize
		desired_width = 0.8 * 1024
		desired_height = 0.8 * 600

		font = ImageFont.truetype(self.font, fontsize)
		width, height = self.text_size(image_draw, text, font)
		ratio_width = desired_width / width
		ratio_height = desired_height / height

		ratio = min(ratio_height, ratio_width)
		return int(fontsize * ratio)

	def pil_to_imgmsg(self, image, encodingmap={'L': 'mono8', 'RGB': 'rgb8', 'RGBA': 'rgba8', 'YCbCr': 'yuv422'},
                        PILmode_channels={'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}):
	    rosimage = ROSImage()
	    # adam print 'Channels image.mode: ',PILmode_channels[image.mode]
	    rosimage.encoding = encodingmap[image.mode]
	    (rosimage.width, rosimage.height) = image.size
	    rosimage.step = PILmode_channels[image.mode] * rosimage.width
	    rosimage.data = image.tostring()
	    return rosimage

	def publish_image(self):
		self.image_publisher.publish(self._image)
		

if __name__ == "__main__":
	rospy.init_node("screen_writer")
	writer = ScreenWriter()
	rate = rospy.Rate(30)
	index = 2
	while not rospy.is_shutdown():
		writer.write(str(index))
		index *= 2
		rate.sleep()