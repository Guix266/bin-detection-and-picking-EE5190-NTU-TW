#!/usr/bin/env python


import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


class Initialise_robot(object):
  	"""create the interface to move the robot"""
  	def __init__(self):
	    super(Initialise_robot, self).__init__()
	    ## First initialize `moveit_commander`_ and a `rospy`_ node:
	    moveit_commander.roscpp_initialize(sys.argv)
	    rospy.init_node('script_move',
	                    anonymous=True)

	    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
	    ## the robot:
	    robot = moveit_commander.RobotCommander()

	    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
	    ## to the world surrounding the robot:
	    scene = moveit_commander.PlanningSceneInterface()

	    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
	    ## to one group of joints. 
	    group_name = "manipulator"
	    group = moveit_commander.MoveGroupCommander(group_name)
	    #group.setPlannerId("SBLkConfigDefault")

	    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
	    ## trajectories for RViz to visualize:
	    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
	                                                   moveit_msgs.msg.DisplayTrajectory,
	                                                   queue_size=20)

	    print("\n##### INFO ABOUT THE ROBOT")
	    planning_frame = group.get_planning_frame()
	    print "====== Reference frame: %s" % planning_frame

	    # We can also print the name of the end-effector link for this group:
	    eef_link = group.get_end_effector_link()
	    print "====== End effector: %s" % eef_link

	    # We can get a list of all the groups in the robot:
	    group_names = robot.get_group_names()
	    print "====== Robot Groups:", robot.get_group_names()

	    # Sometimes for debugging it is useful to print the entire state of the
	    # robot:
	    # print "\n============ Printing robot state"
	    # print robot.get_current_state()

	    # Misc variables
	    self.box_name = ''
	    self.robot = robot
	    self.scene = scene
	    self.group = group
	    self.display_trajectory_publisher = display_trajectory_publisher
	    self.planning_frame = planning_frame
	    self.eef_link = eef_link
	    self.group_names = group_names

	def print_c(self):
		print("Current POSITION COORDINATES :")
		current_pose = self.group.get_current_pose().pose
		print(current_pose)


def main():
	
	print "\n[INFO] INITIALISATION OF THE ROBOT (press ctrl-d to exit)"
	ur10 = Initialise_robot()

	ur10.print_c()


if __name__ == '__main__':
	main()