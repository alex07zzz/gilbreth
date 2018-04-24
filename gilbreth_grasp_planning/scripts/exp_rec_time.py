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
import setproctitle
from gilbreth_msgs.msg import ObjectDetection
from gilbreth_msgs.msg import TargetToolPoses
from sensor_msgs.msg import PointCloud2

TOOL_POSE_TOPIC='/gilbreth/target_tool_poses'
OBJ_DETECTION_TOPIC = '/recognition_result_world'
KINECT_PUB_TOPIC='/gilbreth/kinect_points'
RECOG_TIME_TOPIC='/recognition_time'

class RecogMonitor():
    def __init__(self):
        self.obj_dect_sub = rospy.Subscriber(OBJ_DETECTION_TOPIC, ObjectDetection, self.obj_detect_callback)
        self.recognition_time_sub = rospy.Subscriber(RECOG_TIME_TOPIC,Float64,self.rec_time_callback)
	    #self.recognition_time_sub = rospy.Subscriber(RECOG_TIME_TOPIC,Float64,self.rec_time_callback)
        self.kinect_pub_sub = rospy.Subscriber(KINECT_PUB_TOPIC,PointCloud2, self.kinect_pub_callback)
        self.obj_data_count = 0
        self.kinect_data_count = 0
        self.suc_rate = 0.0
        self.rec_time=[]
        self.obj_detected=[]
        self.count = 0
	
    def obj_detect_callback(self,data):
        if data is not None:
            self.obj_data_count = self.obj_data_count + 1
            print "Recognized %d object"% self.obj_data_count
        

    def kinect_pub_callback(self, data):
        if data is not None:
            self.kinect_data_count = self.kinect_data_count +1
            print "Recieved new objected.Total object: %d"% self.kinect_data_count
            self.suc_rate = float(self.obj_data_count) * 100 / float(self.kinect_data_count)
            print "successful rate is %f"% self.suc_rate

                
    def rec_time_callback(self,msg):
	    if msg is not None:
	 	    print "Recognition time is %f" % float(msg.data)
	 	    self.rec_time.append(msg.data)
	 	    self.count = self.count+1
	        
if __name__ == '__main__':
    try:
        rospy.init_node('recognition_monitor')
        setproctitle.setproctitle('recognition_monitor')    
        rate = rospy.Rate(10)
        recog_monitor = RecogMonitor()
        while not rospy.is_shutdown():
            if recog_monitor.count == 500:
                np.savetxt('rec_disk_1core',recog_monitor.rec_time,delimiter=',')
                rospy.signal_shutdown('no comment')
    except rospy.ROSInterruptException: 
        pass
