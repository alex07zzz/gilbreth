#!/usr/bin/env python
## a python script compute the tool poses
import rospy
import sys
import psutil
import numpy as np
import setproctitle
import time
from std_msgs.msg import Float64
from gilbreth_msgs.msg import ObjectDetection
from gilbreth_msgs.msg import TargetToolPoses
from gilbreth_gazebo.msg import Proximity
 
OBJ_DETECTED_TOPIC = 'break_beam_sensor_change'
OBJ_RECOGNIZED_TOPIC = '/recognition_result_world'
RECOG_TIME_TOPIC='/recognition_time'

object_name = sys.argv[1]
speed = sys.argv[2]

class RecogMonitor():
    def __init__(self):
    	self.obj_detected_sub = rospy.Subscriber(OBJ_DETECTED_TOPIC,Proximity,self.obj_detected_callback)
        self.obj_recognized_sub = rospy.Subscriber(OBJ_RECOGNIZED_TOPIC, ObjectDetection, self.obj_recognized_callback)
        self.recognition_time_sub = rospy.Subscriber(RECOG_TIME_TOPIC,Float64,self.recog_time_callback)
        self.obj_detected_count = 0
        self.obj_recognized_count = 0
        self.success_rate = 0.0
        self.rec_time=[]
        self.part_name = object_name
        self.count = 0

    def obj_detected_callback(self,msg):
	    if msg.object_detected:
	        self.obj_detected_count = self.obj_detected_count + 1
	        print "Recieved new object, total object: %d" %self.obj_detected_count
	        self.count = self.count +1
	    
    def obj_recognized_callback(self,msg):
        if msg.name == self.part_name:
            self.obj_recognized_count = self.obj_recognized_count + 1
            print "Recognized %d object" % self.obj_recognized_count
                
    def recog_time_callback(self,msg):
	    if msg is not None:
	 	    #print "Recognition time is %f" % float(msg.data)
	 	    self.rec_time.append(msg.data)
	        
if __name__ == '__main__':
    try:
        rospy.init_node('recognition_monitor')
        setproctitle.setproctitle('recognition_monitor')    
        rate = rospy.Rate(10)
        #obj_name=sys.argv[1]
        recog_monitor = RecogMonitor()
        while not rospy.is_shutdown():
            if recog_monitor.count == 200:
                fileb = '.npy'
                filename = time.ctime()+'_vel_'+speed+'_'+recog_monitor.part_name+'_'+str(recog_monitor.obj_recognized_count) +'over'+str(recog_monitor.obj_detected_count)+fileb
                np.save(filename,recog_monitor.rec_time)
                rospy.loginfo("Saving files")
                rospy.signal_shutdown('no comment')
    except rospy.ROSInterruptException: 
        pass
