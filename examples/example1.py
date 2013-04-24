#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    person1 = W.add_object("person-1", "person", 2, 0, 0)
    person2 = W.add_object("person-2", "person")
    person2.set_position(2, 1, 0)

    raw_input("Press Enter to continue...")
    person1.set_position(2, 2, 0)