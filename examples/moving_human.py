#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import math

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    human = W.add_object("human", "humanlike_box", 0, 0, 0)
    #coke.set_position(4, 1, 0)
    
    x = 0

    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        x = x + 0.001*3.1415
        human.set_position(0,10*math.sin(x),0)

