#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    #person1 = W.add_object("loy", "person", 2.5, 0.1, 0)
    #coke = W.add_object("coke-1", "coke")
    #coke.set_position(4, 1, 0)

    next_step = False
    while not rospy.is_shutdown() and not next_step:
    	rospy.sleep(0.5)
    	if W.amigo_speech_contains("Which room") or W.amigo_speech_contains("What room") or W.amigo_speech_contains("which room"):
    		rospy.sleep(2.0)
    		W.speak("kitchen", "/speech_room/output")
    		next_step = True

    next_step = False
    while not rospy.is_shutdown() and not next_step:
    	rospy.sleep(0.5)
    	if W.amigo_speech_contains("Am I right") or W.amigo_speech_contains("okay"):
    		rospy.sleep(2.0)
    		W.speak("yes", "/speech_yesno/output")
    		next_step = True
    
    

    #raw_input("Press Enter to continue...")
    #person1.set_position(2, 1.9, 0)
