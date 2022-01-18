#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Byte
from threading import Thread
import time

red_flag=[1]
yellow_flag=[0]
green_flag=[0]
traffic_light_tag=0

def red_led(): 
    if red_flag[0]==1:
        traffic_light_tag=1

def yellow_led():
    if yellow_flag[0]==1:
        traffic_light_tag=2

def green_led():
    if green_flag[0]==1:
        traffic_light_tag=3
    
t1 = Thread(target=red_led, args=red_flag)  
t2 = Thread(target=yellow_led, args=yellow_flag)  
t3 = Thread(target=green_led, args=green_flag)  

t1.start()
t2.start()
t3.start()

traffic_light_pub = rospy.Publisher("/traffic_light_tag", Byte, queue_size=10)

rospy.init_node('traffic_light_control_node', anonymous=True)
rate = rospy.Rate(10) # 10hz
prev_time=rospy.Time.now()
while not rospy.is_shutdown():
        if rospy.Time.now()-prev_time>30:   # 30초 후 green
            red_flag[0]=0
            green_flag[0]=1
            prev_time=rospy.Time.now()
        if rospy.Time.now()-prev_time>5:    # 5초 후 red
            red_flag[0]=1
            green_flag[0]=0
            prev_time=rospy.Time.now()
        traffic_light_pub.publish(traffic_light_tag)
        rate.sleep()

t1.join()
t2.join()
t3.join()


