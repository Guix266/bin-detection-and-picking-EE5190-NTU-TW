#!/usr/bin/env python

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

rospy.init_node('Robotiq2FGripperSimpleController')
pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=20)
rospy.sleep(0.2)
command = outputMsg.Robotiq2FGripper_robot_output()


def initialise_gripper():
    """ Initialise the gripper"""

    # START THE PUBLISHER
    # rospy.init_node('CModelSimpleController')
    
    # command = outputMsg.Robotiq2FGripper_robot_output()

    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 100
    command.rFR  = 20
     
    pub.publish(command)
    rospy.sleep(2)
    print("[INFO] : The gripper is initialised")


def close_gripper():
    """Write the command to close the robot hand and publish them on the CModelRobotOutput topic."""
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=20)
    # command = outputMsg.Robotiq2FGripper_robot_output()
    command.rPR = 255

    pub.publish(command)
    rospy.sleep(2)

def open_gripper():
    """Write the command to close the robot hand and publish them on the CModelRobotOutput topic."""
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=20)
    # command = outputMsg.Robotiq2FGripper_robot_output()
    command.rPR = 0

    pub.publish(command)
    rospy.sleep(2)

                        

if __name__ == '__main__':
    initialise_gripper()
    rospy.sleep(2)
    close_gripper()
    rospy.sleep(2)
    open_gripper()