#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

def wait_for_amigo_speech(texts, type):
    while not rospy.is_shutdown():
        rospy.sleep(0.5)
        for text in texts:
            if W.amigo_speech_contains(text):
                return

if __name__ == "__main__":
    rospy.init_node('egpsr_tester')

    W = client.SimWorld()

    #person1 = W.add_object("loy", "person", 2.5, 0.1, 0)
    #coke = W.add_object("coke-1", "coke")
    #coke.set_position(4, 1, 0)

    W.speak('Carrysomestufftoaseat', 'sentences')
    W.speak("drink", "stuff")
    W.speak("yes", "yesno")
    W.speak("no", "yesno")

    input_sentences = []

    # What can I do for you
    W.wait_for_amigo_speech(["What can I do for you"])
    rospy.sleep(2.0)
    W.speak('Carrysomestufftoaseat', 'sentences')

    # Is that alright?
    W.wait_for_amigo_speech(["Is that alright"])
    rospy.sleep(2.0)
    W.speak('yes', 'yesno')

    # Can you specify which stuff you mean?
    W.wait_for_amigo_speech(["you specify which"])
    rospy.sleep(2.0)
    W.speak('cards', 'stuff')

    # Is that alright?
    W.wait_for_amigo_speech(["Is that alright"])
    rospy.sleep(2.0)
    W.speak('yes', 'yesno')    

    # Are you aware of the room in which I can find a drink
    W.wait_for_amigo_speech(["Are you aware of the room"])
    rospy.sleep(2.0)
    W.speak('no', 'yesno')       

    # Can you specify which seat you mean?
    W.wait_for_amigo_speech(["you specify which"])
    rospy.sleep(2.0)
    W.speak("no", "yesno")

        # I heard no
    # Am I right?
    W.wait_for_amigo_speech(["Am I right", "Is that corect"]) # corect MUST be with 1 r
    rospy.sleep(2.0)
    W.speak('yes', 'yesno')   
