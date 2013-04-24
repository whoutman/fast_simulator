#!/usr/bin/env python
import roslib; roslib.load_manifest('fast_simulator')

import rospy

import fast_simulator.srv
import geometry_msgs.msg
import std_msgs.msg

class SimWorld(object):
	def __init__(self):
		rospy.wait_for_service("/fast_simulator/set_object")

		self.srv_set = rospy.ServiceProxy("/fast_simulator/set_object", fast_simulator.srv.SetObject)

		self.pub_speech = rospy.Publisher("/speech/output", std_msgs.msg.String)

	def add_object(self, id, type, x=None, y=None, z=None):
		obj = SimObject(id, type, self)

		if x:
			obj.set_position(x, y, z)

		return obj

	def speak(self, text):
		self.pub_speech.publish(text)

class SimObject(object):
	def __init__(self, id, type, world):
		self.id = id
		self.type = type
		self.world = world

	def set_position(self, x, y, z):
		req = fast_simulator.srv.SetObjectRequest()

		req.id = self.id
		req.type = self.type
		req.action = fast_simulator.srv.SetObjectRequest.SET_POSE

		req.pose.position.x = x
		req.pose.position.y = y
		req.pose.position.z = z
		req.pose.orientation.x = 0
		req.pose.orientation.y = 0
		req.pose.orientation.z = 0
		req.pose.orientation.w = 1		

		self.world.srv_set(req)


	def set_path(self, path, frame_id="/map"):
		if not path:
			return

		req = fast_simulator.srv.SetObjectRequest()
		req.id = self.id
		req.action = fast_simulator.srv.SetObjectRequest.SET_PATH

		for i in range(0, len(path)):
			waypoint = geometry_msgs.msg.Pose()

			waypoint.position.x = path[i][0]
			waypoint.position.y = path[i][1]
			waypoint.position.z = path[i][2]

			waypoint.orientation.x = 0
			waypoint.orientation.y = 0
			waypoint.orientation.z = 0
			waypoint.orientation.w = 1

			req.path += [waypoint]

		self.world.srv_set(req)

	def set_velocity(self, vel):
		req = fast_simulator.srv.SetObjectRequest()
		req.id = self.id
		req.action = fast_simulator.srv.SetObjectRequest.SET_PARAMS
		req.param_names = ["velocity"]
		req.param_values = [vel]
		self.world.srv_set(req)
