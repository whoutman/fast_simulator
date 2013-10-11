#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import math
import random

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('gazebo_life_example_1')

    W = client.SimWorld()

    human_1 = W.add_object("human_hallway", "humanlike_box", 0.0, 0.0, 0.0)
    human_2 = W.add_object("human_kitchen", "humanlike_box_2", 0.0, 0.0, 0.0)
    human_3 = W.add_object("human_side_corridor_right", "humanlike_box_2", 0.0, 0.0, 0.0)
    human_4 = W.add_object("human_side_corridor_left", "humanlike_box_2", 0.0, 0.0, 0.0)
    human_5 = W.add_object("human_livingroom", "humanlike_box_2", 0.0, 0.0, 0.0)

    v_const = 1.0   
    dec = 1.0

    human = [human_1,human_2,human_3,human_4,human_5]
    
    human_1_xydir = [2.7, -2.0  , 1  , 0 ]
    human_2_xydir = [11.5, -5.1 , -1 , 1 ]
    human_3_xydir = [4.0 , -9.4 , -1 , 1 ]
    human_4_xydir = [4.0 , 2.3  , -1 , 1 ]
    human_5_xydir = [9.5 , -3.0 , -1 , 1 ]
    
    v = [v_const]*len(human)
    
    xydir = [human_1_xydir,human_2_xydir,human_3_xydir,human_4_xydir,human_5_xydir]
               
    x = [0.0]*len(human)
    
    
    time_now   = rospy.get_rostime()
    time_begin = [time_now]*len(human)
    t_last     = [time_now]*len(human)
    t_step     = [0]*len(human)
    
    path_dist  = [2.0,5.0,6.0,6.0,9.5]
    
    time_out        = [rospy.Duration(0)]*len(human)
    random_time_out = [rospy.Duration(0)]*len(human)
    fixed_time_out  = [rospy.Duration(5),
						rospy.Duration(10),
						rospy.Duration(5),
						rospy.Duration(5),
						rospy.Duration(10)]
    t_last_time_out = [time_now]*len(human)

    while not rospy.is_shutdown():
           
        for i in range(len(x)):
                               
            time = rospy.get_rostime()
            
            if time - t_last_time_out[i] > time_out[i]:
                
                random_time_out[i] = rospy.Duration(0)
                   
                t_step[i] = (time - t_last[i]).secs + (time - t_last[i]).nsecs/1e9
                #rospy.loginfo("t_step = {}".format((time - t_last).secs + (time - t_last).nsecs/1e9))       
            
                if x[i]>=path_dist[i]:
                    v[i] = max(0.0,v[i] - dec*t_step[i])
                else:
                    v[i] = v_const

                x[i] = x[i] + t_step[i]*v[i]
                
                #rospy.logwarn("time = {0} | time_begin = {1}".format(time,time_begin[i]))       
                #rospy.logwarn("time - time_begin = {0}".format(time-time_begin[i]))

                if x[i] > path_dist[i] + (v_const*v_const/2*dec) - 0.05:
                    x[i] = 0.0
                    time_begin[i] = rospy.get_rostime()
                    random_time_out[i] = rospy.Duration(random.randint(0,5))
                    time_out[i] = random_time_out[i] + fixed_time_out[i]
                    t_last_time_out[i] = time

                human[i].set_position(xydir[i][0] + xydir[i][2]*xydir[i][3]*x[i],
                                      xydir[i][1] + xydir[i][2]*(1-xydir[i][3])*x[i],
                                      0.0)
             
            t_last[i] = time
            rospy.sleep(0.01)
            
        
        
        


