#!/usr/bin/env python
import roslib; roslib.load_manifest('fast_simulator')

import rospy

import fast_simulator.srv
import geometry_msgs.msg
import std_msgs.msg

class SimWorld(object):
	def __init__(self):
		#rospy.wait_for_service("/fast_simulator/set_object")

		self.srv_set = rospy.ServiceProxy("/fast_simulator/set_object", fast_simulator.srv.SetObject)

		self.pub_speech = rospy.Publisher("/speech/output", std_msgs.msg.String)

		self.sub_amigo_speech = rospy.Subscriber("/amigo_speech_sim", std_msgs.msg.String, self.callback_amigo_speech)
		self.amigo_sentence = ""
		self.t_amigo_sentence = rospy.Time.now()

		# init all amigo hear publishers
		self.output_publishers = {} 
		output_topics = [	"/speech/output", "/speech_appliance/output", "/speech_cleanup/output", "/speech_continue/output",
							"/speech_decoration/output", "/speech_diningroom/output", "/speech_drink/output", "/speech_drink_cocktail/output",
							"/speech_food/output", "/speech_kitchen/output", "/speech_livingroom/output", "/speech_medicine/output",
							"/speech_name/output", "/speech_open_challenge/output", "/speech_room/output", "/speech_seat/output",
							"/speech_sentences/output", "/speech_storage/output", "/speech_stuff/output", "/speech_table/output",
							"/speech_yesno/output"]

		for output_topic in output_topics:
			self.output_publishers[output_topic] = rospy.Publisher(output_topic, std_msgs.msg.String)

	def add_object(self, id, type, x=None, y=None, z=None):
		obj = SimObject(id, type, self)

		if x:
			obj.set_position(x, y, z)

		return obj

	def speak(self, text, type=None):

		if type == None:
			topic = "/speech/output"
		else:
			topic = "/speech_" + type + "/output"

		if not self.output_publishers.has_key(topic):
			rospy.logerr("Unknown amigo speak topic: %s" % topic)
		else:
			print "I say:      " + str(text)
			print
			self.output_publishers[topic].publish(text)

	def callback_amigo_speech(self, string_msg):
		print "AMIGO says: %s" % string_msg.data
		print
		self.amigo_sentence = string_msg.data
		self.t_amigo_sentence = rospy.Time.now()

	def amigo_speech_contains(self, substr, max_dt = rospy.Duration(2.0)):
			return substr in self.amigo_sentence and rospy.Time.now() - self.t_amigo_sentence < max_dt

	def wait_for_amigo_speech(self, texts):
	    while not rospy.is_shutdown():
	        rospy.sleep(0.5)
	        for text in texts:
	            if self.amigo_speech_contains(text):
	                return


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
