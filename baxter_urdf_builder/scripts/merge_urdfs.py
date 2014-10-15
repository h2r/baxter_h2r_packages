#!/usr/bin/env python

import roslib
import rospy
import xml.etree.ElementTree as ET

import copy
import sys

links_to_pull_from_param = []
joints_to_pull_from_param = ["left_torso_arm_mount" , "right_torso_arm_mount" ]

def get_param_urdf():
	urdf_text = rospy.get_param("/robot_description")
	return ET.fromstring(urdf_text)

def load_file_urdf(filename):
	return ET.parse(filename)

def get_element_from_root_node(root, name_of_element):
	for element in root:
		if "name" in element.keys():
			if element.get("name") is name_of_element:
				return element
	return None

def merge_roots(param_root, file_root, elements_to_pull_from_param):
	new_elements = []
	for element in file_root:
		name = element.get("name")
		new_element = copy.deepcopy(element)
		if name in elements_to_pull_from_param:
			param_element = get_element_from_root_node(param_root, name)
			if param_element != None:
				new_element = copy.deepcopy(param_element)
		new_elements.append(new_element)
	return new_elements

def merge_urdf_files(param_urdf, file_urdf):
	param_joints = param_urdf.iterfind("joint")
	param_links = param_urdf.iterfind("link")

	file_joints = file_urdf.iterfind("joint")
	file_links = file_urdf.iterfind("link")

	new_joints = merge_roots(param_joints, file_joints, joints_to_pull_from_param)
	new_links = merge_roots(param_links, file_links, links_to_pull_from_param)

	new_root = ET.Element(file_urdf.tag, file_urdf.attrib)

	for link in new_links:
		new_root.append(link)

	for joint in new_joints:
		new_root.append(joint)
	
	return new_root



def write_urdf(urdf, filename):
	tree = ET.ElementTree(urdf)
	tree.write(filename)


if __name__ == "__main__":
	rospy.init_node("urdf_merger")
	args = rospy.myargv(sys.argv)
	urdf_filename = args[1]
	output_filename = args[2]
	urdf_tree = get_param_urdf()
	file_tree = load_file_urdf(urdf_filename)

	new_root = merge_urdf_files(urdf_tree, file_tree.getroot())
	write_urdf(new_root, output_filename)