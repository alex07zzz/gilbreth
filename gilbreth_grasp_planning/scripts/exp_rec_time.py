#!/usr/bin/env python
## a python script compute the tool poses
import rospy
import sys
import rospy
import math
import copy
import psutil
import timeit
import numpy as np
import csv
from std_msgs.msg import Float64

rec_time=[]
count = 0
#w = open("rec_disk","w")
def rec_callback(msg):
    print msg
    global count
    count += 1
    rec_time.append(msg.data)
    print count
    

if __name__ == '__main__':
    try:
        rospy.init_node('recognition_monitor')
        rate = rospy.Rate(10)
        rospy.Subscriber('/recognition_time',Float64,rec_callback)
        while not rospy.is_shutdown():
            if count == 50:
                np.savetxt('rec_disk_1core',rec_time,delimiter=',')
                rospy.signal_shutdown('no comment')
    except rospy.ROSInterruptException: 
        pass
