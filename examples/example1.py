#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    person1 = W.add_object("loy", "person", 2.5, 0.1, 0)
    coke = W.add_object("coke-1", "coke")
    coke.set_position(4, 1, 0)

    raw_input("Press Enter to continue...")
    person1.set_position(2, 1.9, 0)
