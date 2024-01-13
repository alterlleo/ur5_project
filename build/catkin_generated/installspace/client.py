#!/usr/bin/env python3

# from __future__ import print_function

import sys
import time, json

import rospy
import numpy as np
from numpy import pi
from robotics_project.srv import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from geometry_msgs.msg import Quaternion, Point, Pose


def rpy2quaternion(roll, pitch, yaw):
	qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
	qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
	qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
	qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
	return qx, qy, qz, qw


def transform_position(x, y, z):
	return 1.0 - x, 0.8 - y, z + 0.87


def yolo_client():
	rospy.wait_for_service('vision')
	try:
		yolo_detection = rospy.ServiceProxy('vision', YoloResults)
		resp1 = yolo_detection()
		return resp1.x
	except rospy.ServiceException as e:
		print("Service call failed: %s" % e)


def move_model(model_name, position, orientation):
	state_msg = ModelState()
	state_msg.model_name = model_name
	state_msg.pose.position = Point(*transform_position(position[0], position[1], position[2]))
	state_msg.pose.orientation = Quaternion(*rpy2quaternion(orientation[0], orientation[1], orientation[2]))
	rospy.wait_for_service('/gazebo/set_model_state')
	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	res = set_state(state_msg)
	return res


if __name__ == "__main__":
	# model_name = "X1-Y1-Z2"
	nx = 15
	ny = 10
	minx = 0.01
	maxx = 0.99
	miny = 0.02
	maxy = 0.62

	# position = np.array([0.95, 0.42, 0.012])
	# orientation = np.array([0.0, 0.0, 0.0])
	# success = move_model(model_name, position, orientation)
	# time.sleep(1)
	res = yolo_client()

	# print("sent", position[0], position[1])
	print("recieved", res)

	# measures = []
	# for i in range(nx):
	# 	for j in range(ny):
	# 		xpos = (maxx - minx) / (nx - 1) * i + minx
	# 		ypos = (maxy - miny) / (ny - 1) * j + miny
	# 		position = np.array([xpos, ypos, 0.012])
	# 		orientation = np.array([0.0, 0.0, 0.0])
	# 		success = move_model(model_name, position, orientation)
	# 		time.sleep(1)
	# 		res = yolo_client()
	# 		measure = {"pos": [xpos, ypos], "centre": [res[0], res[1]]}
	# 		measures.append(measure)
	# with open("/home/alessandro/ros_ws/src/robotics_project/scripts/Yolo/measures.json", "w") as f:
	# 	f.write(json.dumps(measures))
