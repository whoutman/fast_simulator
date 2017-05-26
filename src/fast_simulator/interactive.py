#! /usr/bin/env python

import rospy
import sys
from fast_simulator import client

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

if __name__ == "__main__":
    rospy.init_node("interactive_simobject")

    server = InteractiveMarkerServer("interactive_simobject")
    rospy.loginfo("InteractiveMarkerServer created")

    W = client.SimWorld()
    rospy.loginfo("SimWorld connected")

    object_id = sys.argv[1]
    object_type = sys.argv[2]
    x, y, z = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])

    sim_object = W.add_object(object_id, object_type, x, y, z)

    # create an interactive marker for our server
    interactive_marker = InteractiveMarker()
    interactive_marker.header.frame_id = "/map"
    interactive_marker.name = object_id
    interactive_marker.description = "Control" + object_id

    # Create sphere as 'center' of the control
    box_marker = Marker()
    box_marker.type = Marker.SPHERE
    box_marker.scale.x = 0.1
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 1.0
    box_marker.color.g = 0.2
    box_marker.color.b = 0.2
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    # add the control to the interactive marker
    interactive_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    translate_x = InteractiveMarkerControl()
    translate_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    translate_x.orientation.w = 1
    translate_x.orientation.x = 1
    translate_x.orientation.y = 0
    translate_x.orientation.z = 0
    translate_x.name = "translate_x"
    interactive_marker.controls.append(translate_x)

    translate_y = InteractiveMarkerControl()
    translate_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    translate_y.orientation.w = 1
    translate_y.orientation.x = 0
    translate_y.orientation.y = 0
    translate_y.orientation.z = 1
    translate_y.name = "translate_y"
    interactive_marker.controls.append(translate_y)

    translate_z = InteractiveMarkerControl()
    translate_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    translate_z.orientation.w = 1
    translate_z.orientation.x = 0
    translate_z.orientation.y = 1
    translate_z.orientation.z = 0
    translate_z.name = "translate_z"
    interactive_marker.controls.append(translate_z)

    def update_object(feedback):
        p = feedback.pose.position
        sim_object.set_position(p.x, p.y, p.z)
        print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

    # add the interactive marker to our collection &
    # tell the server to call update_object() when feedback arrives for it
    server.insert(interactive_marker, update_object)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
