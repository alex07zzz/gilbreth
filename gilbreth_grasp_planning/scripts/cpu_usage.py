#!/usr/bin/python
import rospy
import psutil
from std_msgs.msg import String
import sys
from os.path import expanduser
import multiprocessing as mp
import numpy as np
import os
import thread
import csv
import string
import time
import setproctitle

class cpu_usage:
    def __init__(self):
        self.process_name_list = ['move_group','gzserver','robot_execution','tool_planner','recognition_node','segmentation_node']
        self.pid_list=[]

    def find_pid_by_name(self):
        for name in self.process_name_list:
            for p in psutil.process_iter(attrs=['name']):
                if p.info['name']==name:
                    self.pid_list.append(p.pid)
#        print self.pid_list  
        return self.pid_list

    def proc_cpu_usage(self,pid,total_t,start_time,ros_time):
        proc = psutil.Process(pid)
        pid_info = proc.as_dict(attrs=['name','pid'])
        #print pid_info
        cpu_usage = []
        cpu_time = []
        count = 0
        while count < total_t:
            usage = proc.cpu_percent(interval= 0.1)
            cpu_usage.append(usage)
            cpu_time.append(time.time())
            count = count + 1
        cpu_time[:]=[x-start_time + ros_time for x in cpu_time]
        pid_info.update({"cpu_usage":cpu_usage,"time":cpu_time})
        name = str(pid)
        fileb = '.npy'
        filename = time.ctime()+'_'+pid_info['name']+'_'+name +fileb
        np.save(filename,pid_info)
        print pid_info  
        return pid_info


def main(args):
    rospy.init_node('ros_cpu_monitor')
    rate = rospy.Rate(10)
    cpu_monitor=cpu_usage()
    cpu_monitor.find_pid_by_name()
    print cpu_monitor.pid_list
    start_time = time.time()
    begin_ros_time = rospy.Time.now().to_sec()
    for pid in cpu_monitor.pid_list:
        p=mp.Process(target=cpu_monitor.proc_cpu_usage,args=(pid,1000,start_time,begin_ros_time))
        p.start()

if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
