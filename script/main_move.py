#!/usr/bin/env python

import script_move
from std_msgs.msg import String,Float64

pi = 3.14159265359 

###################### Gripper code
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

rospy.init_node('Controler_move_robot')
pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=20)
rospy.sleep(0.2)
command = outputMsg.Robotiq2FGripper_robot_output()


def initialise_gripper():
    """ Initialise the gripper"""
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 20
     
    pub.publish(command)
    rospy.sleep(0.5)
    print("[INFO] : The gripper is initialised")

def reset():
    """Write the command to close the robot hand and publish them on the CModelRobotOutput topic."""
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.5)


def close_gripper():
    """Write the command to close the robot hand and publish them on the CModelRobotOutput topic."""
    command.rPR = 255
    pub.publish(command)
    rospy.sleep(0.5)

def open_gripper():
    """Write the command to close the robot hand and publish them on the CModelRobotOutput topic."""
    command.rPR = 0
    pub.publish(command)
    rospy.sleep(0.5)

##################### Initialise move_it for the robot
print "\n[INFO] INITIALISATION OF THE ROBOT (press ctrl-d to exit)"
ur10 = script_move.Initialise_robot()
initialise_gripper()



##################### Get the message of the object position

def move_cube(message):
	print("The message receive is : ")
	print(message)
	print("\n")

	a,cube_x, cube_y, angle, b = str(message).split(",")

	cube_x = float(str(cube_x)) - 0.025
	cube_y = float(str(cube_y)) + 0.586
	angle = float(str(angle))
	
	initialise_gripper()

	result = ur10.go_starting_point()
	print(u"The robot is ready = %r" % str(result))

	# Rotate
	ur10.rotation(-angle)

	result = ur10.pick_object(cube_x,cube_y)
	print(u"The robot reached the point = %r" % str(result))


	close_gripper()

	result = ur10.lift_up_the_object(-0.1,0.5, z=0.5)
	print(u"The robot reached the point = %r" % str(result))

	open_gripper()

	rospy.sleep(5)

	reset()

rospy.Subscriber("Object_cordinate", String, move_cube)
rospy.spin()
















		# print "\n[INFO] INITIALISATION OF THE ROBOT (press ctrl-d to exit)"
		# ur10 = script_move.Initialise_robot()
		# initialise_gripper()

		# print "\n##### (press key) Set the robot in starting position :"
		# raw_input()
		# result = ur10.go_starting_point()
		# print(u"The robot is ready = %r" % str(result))

		# # ur10.rotation(pi)

		# close_gripper()

		# open_gripper()
		# print "\n##### (press key) Change the position of the end effector with a Cartesian path"

		# raw_input()
		# result = ur10.sm.pick_object(0,0.9)
		# print(u"The robot reached the point = %r" % str(result))

		# gc.close_gripper()

		# result = ur10.sm.lift_up_the_object(0.3,0.5)
		# print(u"The robot reached the point = %r" % str(result))

		# gc.open_gripper()

