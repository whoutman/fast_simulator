#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import math

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    #human = W.add_object("human", "humanlike_box", 0, 0, 0)
    human = W.add_object("human", "humanlike_box2", 0, 0, 0)
    #coke.set_position(4, 1, 0)

    #velocity
    v_const = 1.2
    v = v_const
    dec = 1

    x = 0
    t_step = 0.01

    time_out = rospy.Duration(10)
    time = rospy.Time.now()

    #timing for collision
    #rospy.sleep(18.0)
    #rospy.sleep(13.0)

    while not rospy.is_shutdown():

        rospy.sleep(t_step)

        if x>=9.1:
            v = max(0,v - dec*t_step)
        else:
            time = rospy.Time.now()
            v = v_const

        x = x + t_step*v

        if rospy.Time.now() - time > time_out:
            x = 0
            human.set_position(9.5-x,-3,0)
            #rospy.sleep(9)

        #human.set_position(-0.5,-3-x,0)
        human.set_position(9.5-x,-3,0)


