#!/usr/bin/env python

"""
gripper.py
Gripper control node for RoboPuppet and Baxter
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
import baxter_interface as baxter
from interface import Interface
from baxter_interface import CHECK_VERSION

"""
Class Definition
"""
class Gripper:

	def __init__(self):
		"""
		Initializes gripper controller
		"""
		
		# Init ROS node
		rospy.init_node('gripper')
		arm_side = rospy.get_param('~arm_side')
		self._puppet = Interface(arm_side)
		
		# Init baxter gripper controller
		baxter.RobotEnable(CHECK_VERSION).enable()
		limb_name = ('left' if arm_side == 'L' else 'right')
		self._gripper = baxter.Gripper(limb_name)
		self._gripper.open()
	
	def update(self):
		"""
		Updates gripper
		:return: None
		"""
		if self._puppet.get_gripper(0) > 0.5:
			self._gripper.close()
		else:
			self._gripper.open()

"""
Main Function
"""
if __name__ == '__main__':
	node = Gripper()
	comm_rate = rospy.get_param('~comm_rate')
	rate = rospy.Rate(comm_rate)
	while not rospy.is_shutdown():
		node.update()
		rate.sleep()

