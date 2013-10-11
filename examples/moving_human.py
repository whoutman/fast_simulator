#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import math
import random
import tf

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('moving_human_model_example')
    tf_listener = tf.TransformListener()

    W = client.SimWorld()

    human = W.add_object("human_hallway", "humanlike_box_2", 0.0, 0.0, 0.0)

    v_const = 1.0   
    dec = 1.0
    
    xydir = [2.0, -3.0  , -1  , 1 ]
    
    v = v_const
                   
    x = 0.0
    
    time_now   = rospy.get_rostime()
    time_begin = time_now
    t_last     = time_now
    t_step     = 0
    
    path_dist  = 2.0
    
    time_out        = rospy.Duration(5)
    random_time_out = rospy.Duration(0)
    fixed_time_out  = rospy.Duration(0)
    t_last_time_out = time_now
    t_stop 			= time_now
    
    trigger = 1;

    while not rospy.is_shutdown():
            
            try:
                time = rospy.Time()#.now()
                tf_listener.waitForTransform("/map", "/amigo/base_link", time, rospy.Duration(2.0))
                (trans,rot) = tf_listener.lookupTransform("/map", "/amigo/base_link", time)
            except (tf.LookupException, tf.ConnectivityException):
                rospy.logerr("tf request failed!!!")    
                
            time = rospy.Time.now() 
            
                                           
            #if time - t_last_time_out > time_out:
            if trans[1] > -5.0 and trigger == 1:
                                  
                t_step = (time-t_last).secs + (time-t_last).nsecs/1e9
            
                if x>=path_dist:
                    v = max(0.0,v - dec*t_step)
                else:
                    v = v_const

                x = x + t_step*v

                if x > path_dist + (v_const*v_const/2*dec) - 0.05:
                    trigger = 0;
                    #x = 0.0
                    #time_begin = rospy.get_rostime()
                    #time_out= fixed_time_out
                    t_stop = time
                    
                human.set_position( xydir[0] + xydir[2]*xydir[3]*x,
                                    xydir[1] + xydir[2]*(1-xydir[3])*x,
                                    0.0 )
                                                   
            if trigger == 0 and (time - t_stop > time_out):
                x = 0.0
                human.set_position( xydir[0] + xydir[2]*xydir[3]*x,
                                    xydir[1] + xydir[2]*(1-xydir[3])*x,
                                    0.0 )
             
            t_last = time
            rospy.sleep(0.01)
            
        
        
        


