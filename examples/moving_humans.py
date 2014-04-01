#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import math

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    human = W.add_object("human", "humanlike_box", 0.0, 0.0, 0.0)
    human_2 = W.add_object("human_2", "humanlike_box_2", 0.0, 0.0, 0.0)
    human_corridor_1 = W.add_object("human_corridor_1", "humanlike_box_2", 0.0, 0.0, 0.0)
    human_corridor_2 = W.add_object("human_corridor_2", "humanlike_box_2", 0.0, 0.0, 0.0)
    #coke.set_position(4, 1, 0)

    #velocity
    v_const = 1.0
    v = v_const
    dec = 1.0

    x = 0.0
    t_step = 0.03

    time_out = rospy.Duration(10.0)
    time_begin = rospy.get_rostime()
    t_last = time_begin

    while not rospy.is_shutdown():
           
        time = rospy.get_rostime()
        
        t_step = (time - t_last).secs + (time - t_last).nsecs/1e9
        #rospy.loginfo("t_step = {}".format((time - t_last).secs + (time - t_last).nsecs/1e9))

        if x>=9.1:
            v = max(0.0,v - dec*t_step)
        else:
            v = v_const

        x = x + t_step*v
        #rospy.logwarn("x = {}".format(x))

        if time - time_begin > time_out:
            x = 0.0
            time_begin = rospy.get_rostime()

        human.set_position(-0.5,-9+x,0.0)
        human_2.set_position(10.0-x,-5.1,0.0)
        human_corridor_1.set_position(9.0-x,-9.4,0.0)
        human_corridor_2.set_position(9.0-x,2.3,0.0)
             
        t_last = time
        rospy.sleep(0.01)
        
        


