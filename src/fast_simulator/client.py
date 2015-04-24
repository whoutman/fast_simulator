#!/usr/bin/env python
import roslib; roslib.load_manifest('fast_simulator')

import rospy

import fast_simulator.srv
import geometry_msgs.msg
import std_msgs.msg

import tf

class SimWorld(object):
    def __init__(self):
        rospy.wait_for_service("/fast_simulator/set_object")
        self.srv_set = rospy.ServiceProxy("/fast_simulator/set_object", fast_simulator.srv.SetObject)

        self.pub_speech = rospy.Publisher("/pocketsphinx/output", std_msgs.msg.String, queue_size=10)

        self.sub_amigo_speech = rospy.Subscriber("/amigo_speech_sim", std_msgs.msg.String, self.callback_amigo_speech) 
        self.amigo_sentence = ""
        self.t_amigo_sentence = rospy.Time.now()
        
        self.show_amigo_speech = True

        self.objects = {}
        # self.speechmatchers = {}

    def add_object(self, id, type, x=None, y=None, z=None, rx=0, ry=0, rz=0):
        obj = SimObject(id, type, self)
        self.objects[id] = obj

        if x:
            obj.set_position(x, y, z, rx, ry, rz)

        return obj

    def get_object(self, id):
        return SimObject(id, "", self)

        #f not id in self.objects.keys():
        #   return None
        #lse:
        #   return self.objects[id]

    def speak(self, text, type=None):
        print "I say:      " + str(text)
        self.pub_speech.publish(text)

    def callback_amigo_speech(self, string_msg):
        if self.show_amigo_speech:
            print "AMIGO says: %s" % string_msg.data
        if string_msg.data != '':
            self.amigo_sentence = string_msg.data
            self.t_amigo_sentence = rospy.Time.now()

    def amigo_speech_contains(self, substr, max_dt = rospy.Duration(2.0)):
            return substr in self.amigo_sentence and rospy.Time.now() - self.t_amigo_sentence < max_dt

    def wait_for_amigo_speech(self, matchtargets, max_dt = rospy.Duration(2.0)):
        """matchtargets is a list of strings *and* functions. When none of the strings match,
        this method returns when one of the matchfunctions returns True"""
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            strings = [text for text in matchtargets if type(text) == str]
            for text in strings:
                if self.amigo_speech_contains(text, max_dt=max_dt):
                    return self.amigo_sentence
            matchfuncs = [func for func in matchtargets if callable(func)]
            for func in matchfuncs:
                if func(self.amigo_sentence)  and rospy.Time.now() - self.t_amigo_sentence < max_dt:
                    return self.amigo_sentence

    # def add_response(self, matchfunc, response):
    #     """Add a function to match sentences.
    #        When the received sentence matches according to matchfunc, then self.tts replies with 'response'"""
    #     self.speechmatchers[matchfunc] = response

    # def tts(self, robot_text, language='en'):
    #     rospy.sleep(self.delay_factor*len(robot_text))
        
    #     matches = [response for matchfunc, response in self.matchers if matchfunc(robot_text)]

    #     try:
    #         human_text = matches[0]
    #         self.pub.publish(human_text)
    #     except IndexError:
    #         rospy.logwarn("No matching response for '{0}'".format(robot_text))


class SimObject(object):
    def __init__(self, id, type, world):
        self.id = id
        self.type = type
        self.world = world

    def set_position(self, x, y, z, rx = 0, ry = 0, rz = 0):
        req = fast_simulator.srv.SetObjectRequest()

        req.id = self.id
        req.type = self.type
        req.action = fast_simulator.srv.SetObjectRequest.SET_POSE

        q = tf.transformations.quaternion_from_euler(rx, ry, rz)

        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.pose.orientation.x = q[0]
        req.pose.orientation.y = q[1]
        req.pose.orientation.z = q[2]
        req.pose.orientation.w = q[3]      

        if not self.world.srv_set(req):
            rospy.roserr("Service call failed")

    def set_parameter(self, name, value):
        req = fast_simulator.srv.SetObjectRequest()
        req.id = self.id
        req.action = fast_simulator.srv.SetObjectRequest.SET_PARAMS
        req.param_names = [name]
        req.param_values = [value]
        self.world.srv_set(req)

    def remove(self):
        req = fast_simulator.srv.SetObjectRequest()
        req.id = self.id
        req.type = self.type
        req.action = fast_simulator.srv.SetObjectRequest.DELETE

        if not self.world.srv_set(req):
            rospy.roserr("Service call failed")



    def set_path(self, path, path_vel, frame_id="/map"):
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

        req.path_velocity = path_vel

        self.world.srv_set(req)

    def set_velocity(self, vel):
        req = fast_simulator.srv.SetObjectRequest()
        req.id = self.id
        req.action = fast_simulator.srv.SetObjectRequest.SET_PARAMS
        req.param_names = ["velocity"]
        req.param_values = [vel]
        self.world.srv_set(req)
