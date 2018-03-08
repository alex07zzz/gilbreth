#!/usr/bin/env python
## A node executes the robot. 
## Including Gripper control.

import sys
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import time

from gilbreth_msgs.msg import RobotTrajectories
from gilbreth_msgs.msg import TargetToolPoses
from gilbreth_gazebo.msg import VacuumGripperState
from gilbreth_gazebo.srv import VacuumGripperControl
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose

ROBOT_TRAJ_TOPIC='gilbreth/robot_trajectories'
TOOL_POSE_TOPIC='gilbreth/target_tool_poses'
GRIPPER_STATE_TOPIC='gilbreth/gripper/state'
GRIPPER_SERIVE_TOPIC='gilbreth/gripper/control'
#ARM_GROUP_NAME = 'robot'
RAIL_GROUP_NAME = 'robot_rail'
MOVEIT_PLANNING_SERVICE = 'plan_kinematic_path'

def waitForMoveGroup(wait_time = 10.0):
  ready = False
  try:
    rospy.wait_for_service(MOVEIT_PLANNING_SERVICE,wait_time)
    ready = True
  except rospy.ROSException as expt:
    pass

  except rospy.ROSInterruptionException as expt:
    pass
    
  return ready

def curateTrajectory(traj):
  # This is a hack that fixes the issue reported in the link below
  # https://github.com/ros-controls/ros_controllers/issues/291
  if len(traj.joint_trajectory.points) > 0:  
    rospy.logwarn("Trajectory points list is empty")
    traj.joint_trajectory.points[0].time_from_start = rospy.Duration(0.01)
  return traj   

class RobotExecution:
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()        
        #self.arm_group = self.robot.get_group(ARM_GROUP_NAME)
        self.rail_group = self.robot.get_group(RAIL_GROUP_NAME)
        #self.arm_group.set_planning_time(1.0)
        self.rail_group.set_planning_time(1.0)
        self.max_planning_count = 5

        #self.trajectory_sub = rospy.Subscriber(ROBOT_TRAJ_TOPIC, RobotTrajectories, self.trajectory_callback)
        self.tool_sub = rospy.Subscriber(TOOL_POSE_TOPIC,TargetToolPoses,self.tool_poses_callback)
        self.gripper_sub = rospy.Subscriber(GRIPPER_STATE_TOPIC, VacuumGripperState, self.gripper_callback)
        self.gripper_client = rospy.ServiceProxy(GRIPPER_SERIVE_TOPIC, VacuumGripperControl)

        self.gripper_state = VacuumGripperState()
        self.robot_trajectory = RobotTrajectories()
        self.tool_pose = TargetToolPoses()
        self.tool_poses_list = []
        self.EXECUTE = False

        ## define a waiting pose for robot
        self.waiting_pose = Pose()
        self.waiting_pose.position.x = 1.2
        self.waiting_pose.position.y = 0.0
        self.waiting_pose.position.z = 1.2
        self.waiting_pose.orientation.x = 0
        self.waiting_pose.orientation.y = 0.9999999
        self.waiting_pose.orientation.z = 0
        self.waiting_pose.orientation.w = 0

    ## callback functions
    def gripper_callback(self, gripper_data):
        self.gripper_state = gripper_data

    def tool_poses_callback(self, tool_pose_data):
        if tool_pose_data is not None:
            rospy.loginfo('Received new tool pose')
            self.tool_poses_list.append(tool_pose_data)
            self.tool_pose = copy.deepcopy(tool_pose_data)
            self.EXECUTE = True
        else:
            rospy.logerr('Received trajectory is invalid')
          
    ## enable vacuum gripper suction cup
    def enable_gripper(self):
        rospy.loginfo("Enabling Vacuum Gripper...")
        rospy.wait_for_service(GRIPPER_SERIVE_TOPIC)
        try:
            resp = self.gripper_client(enable = True)
            rospy.loginfo("Gripper Enabling Success ")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" %e)

    ## disable vacuum gripper suction cup
    def disable_gripper(self):
        rospy.loginfo("Disabling Vacuum Gripper...")
        rospy.wait_for_service(GRIPPER_SERIVE_TOPIC)
        try:
            resp = self.gripper_client(enable = False)
            rospy.loginfo("Gripper Disabling Success ")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" %e)

    def gripper_attached(self, pick_dead):
        while (rospy.Time.now() < pick_dead + rospy.Duration(2.0)):
            rospy.loginfo('Gripper State: %i'%(self.gripper_state.attached))
            if self.gripper_state.attached:
                return True
            else:
                rospy.sleep(0.2)
        rospy.loginfo("Nothing Attached")
        return self.gripper_state.attached

    def compute_trajectory(self,group):
        for i in range(self.max_planning_count):
            plan = group.plan()
            if len(plan.joint_trajectory.points)>0:
                rospy.loginfo("Motion Plan Success.")
                return plan
            else:
                rospy.logerr("Failed to compute trajectory %d time. Recalculating", i)
        return False

    def goto_waiting_pose(self):
        rospy.loginfo("Start Motion Planning: Current Pose ==> Waiting Pose.")
        self.rail_group.set_start_state_to_current_state()
        self.rail_group.set_pose_target(self.waiting_pose)
 
        waiting_plan = self.compute_trajectory(self.rail_group)
        ## if trajectory is valid, move the robot
        if waiting_plan:
            rospy.loginfo("Motion Plan Success: Current Pose ==> Waiting Pose.")
            if self.rail_group.execute(waiting_plan):
              rospy.loginfo("Moved Robot to Waiting Pose")
        else:
            rospy.logerr("Motion Plan Failed time: Current Pose ==> Waiting Pose.")
    
    def motion_plan(self, group, target_pose):
        rospy.loginfo("Motion Planning...")
        traj= RobotTrajectory()
        #print "traj init"
        group.set_start_state_to_current_state()
        #print "group set"
        group.set_pose_target(target_pose)
        #print "target set"
        traj = self.compute_trajectory(group)
        #print "traj comuted"
        if traj:
            rospy.loginfo("Motion Plan Succeeded. Proceeding...")
            return traj
        else:
            rospy.logerr("Motion Plan Failed. Waiting for next object.")
            return False

    def check_next_obj(self):
        for i in range(len(self.tool_poses_list)):
            if self.tool_poses_list[0].pick_approach.header.stamp > rospy.Time.now():
                rospy.loginfo("Availble object in the list. Proceeding...")                
                return True
            else:
                rospy.loginfo("Not availble. Poping and checking next obj")
                self.tool_poses_list.pop(0)
        rospy.loginfo("No available obj. Waiting for next object...")
        return False
                



    ## execute the robot based on robot_trajectories
    def execute_robot(self):

      class ScopeExit(object):
        def __init__(self,obj):
          self.obj_ = obj

        def __enter__(self):
          return self

        def __exit__(self, exc_type, exc_value, traceback):
          #self.obj_.robot_trajectory = None
          #print "exiting"
          self.tool_pose = None
          self.obj_.EXECUTE = False
          if self.obj_.check_next_obj():
              self.obj_.EXECUTE = True
              print "go to work!!"
          return True

      with ScopeExit(self) as sc:       
                
        if self.EXECUTE:
            target_tool_pose = copy.deepcopy(self.tool_poses_list[0])
            rospy.loginfo("Moving robot from current pose to pick approach pose")
            ## approach trajectory motion plan           
            start_time = time.time()
            #print "go to motion plan"
            app_traj = self.motion_plan(self.rail_group,target_tool_pose.pick_approach.pose)
            rospy.loginfo("Run time is %f seconds",(time.time()-start_time))
            
            if app_traj:
                app_dur = app_traj.joint_trajectory.points[-1].time_from_start
                curateTrajectory(app_traj)
                try:
                    self.rail_group.execute(app_traj)
                    rospy.loginfo("Approaching duration is : %f" % app_dur.to_sec())
                    rospy.sleep(app_dur.to_sec())
                except:
                    rospy.logerr("Failed to execute approach plan...")
                    self.EXECUTE = False
                ## pick trajectory motion plan           
                start_time = time.time()
                pick_traj = self.motion_plan(self.rail_group,target_tool_pose.pick_pose.pose)
                rospy.loginfo("Run time is %f seconds",(time.time()-start_time))

                if pick_traj:
                    pick_dur = pick_traj.joint_trajectory.points[-1].time_from_start
                    pick_deadline = copy.deepcopy(target_tool_pose.pick_pose.header.stamp)
                    curateTrajectory(pick_traj)
    
                    current_time = rospy.Time.now()

                    if (current_time + pick_dur > pick_deadline):
                        rospy.logerr("Can not reach pick pose on time. Drop this picking assignment.")
                        self.EXECUTE = False
                    else:
                        ## Wait to execute robot to pick item
                        rospy.loginfo("Waiting to pick object")
                        current_time = rospy.Time.now()
                        wait_dur = pick_deadline.to_sec()-current_time.to_sec()-pick_dur.to_sec()-0.5
                        rospy.sleep(wait_dur)

                        ## enable gripper for object grasping
                        self.enable_gripper()
                        try:
                            rospy.loginfo("Moving robot from approach pose to pick pose")
                            self.rail_group.execute(pick_traj)
                            rospy.loginfo("Picking duration is : %f" % pick_dur.to_sec())
                            rospy.sleep(pick_dur.to_sec())
                        except:
                            rospy.logerr("Failed to execute pick plan...")
                            self.EXECUTE = False

                        ## check if any object is attached to the vacuum gripper  
                        if self.gripper_attached(pick_deadline):
                            rospy.loginfo("Executing pick to retreat plan")
                            ## retreat trajectory motion plan           
                            start_time = time.time()
                            retreat_traj = self.motion_plan(self.rail_group,target_tool_pose.pick_retreat.pose)
                            rospy.loginfo("Run time is %f seconds",(time.time()-start_time))

                            if retreat_traj:
                                retreat_dur = retreat_traj.joint_trajectory.points[-1].time_from_start                  
                                curateTrajectory(retreat_traj)
                                try:
                                    self.rail_group.execute(retreat_traj)
                                    rospy.loginfo("Retreating duration is : %f" % retreat_dur.to_sec())
                                    rospy.sleep(retreat_dur.to_sec())
                                except:
                                    rospy.logerr("Failed to execute retreat plan")
                                    self.EXECUTE = False
                                rospy.loginfo("Executing retreat to place plan")
                                start_time = time.time()
                                place_traj = self.motion_plan(self.rail_group,target_tool_pose.place_pose.pose)
                                rospy.loginfo("Run time is %f seconds",(time.time()-start_time))

                                if place_traj:
                                    curateTrajectory(place_traj)
                                    try:
                                        self.rail_group.execute(place_traj)
                                        place_dur = place_traj.joint_trajectory.points[-1].time_from_start                  
                                        rospy.loginfo("Retreating duration is : %f" % place_dur.to_sec())
                                        rospy.sleep(retreat_dur.to_sec())
                                    except:
                                        rospy.logerr("Failed to execute place plan...")
                                        self.EXECUTE = False
                                    
                                    self.disable_gripper()
                                    rospy.sleep(1)
                                else:
                                    rospy.logerr("Invalid place trajectory. Go to waiting pose...")
                            else:  
                                rospy.logerr("Invalid retreat trajectory. Go to waiting pose...")
                        else:  
                            rospy.logerr("Nothing attached. Go to waiting pose...")
                            self.disable_gripper()
                else:
                    rospy.logerr("Invalid pick trajectory. Go to waiting pose...")
            else:
                rospy.logerr("Invalid approach trajectory. Go to waiting pose...")
            self.tool_poses_list.pop(0)
            self.disable_gripper()
            if not self.check_next_obj():
                self.goto_waiting_pose()

        
def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('robot_execution',anonymous=True)
    rate = rospy.Rate(10)

    rospy.loginfo("Waiting for move group")
    if waitForMoveGroup():
      rospy.loginfo("Found move group node, proceeding")
    else:
      rospy.logerr("Timed out waiting for move group node, exiting ...")
      sys.exit(-1)

    rb_exec = RobotExecution()
    rb_exec.goto_waiting_pose()

    while not rospy.is_shutdown():
        rb_exec.execute_robot()
        rate.sleep()


if __name__=='__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

