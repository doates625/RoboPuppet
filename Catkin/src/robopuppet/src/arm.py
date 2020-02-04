#!/usr/bin/env python

"""
arm.py
RoboPuppet Baxter arm control node
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
import baxter_interface as baxter
from constants import num_joints
from baxter_interface import CHECK_VERSION
from ros_interface import ROSInterface

"""
Class Definition
"""
class Arm:

	def __init__(self):
		"""
		Constructs arm controller node
		- Enables Baxter
		- Creates Baxter arm interface
		- Creates RoboPuppet ROS interface
		"""
		
		# Init ROS node
		rospy.init_node('arm')
		arm_side = rospy.get_param('~arm_side')
		
		# Enable Baxter
		enabler = baxter.RobotEnable(CHECK_VERSION).enable()
		
		# Baxter arm interface
		limb_name = ('left' if arm_side == 'L' else 'right')
		self._arm = baxter.Limb(limb_name)
		self._joint_names = self._arm.joint_names()
		self._joint_angles = dict()
		for j in range(num_joints):
			self._joint_angles[self._joint_names[j]] = 0.0
		
		# RoboPuppet ROS interface
		self._puppet = ROSInterface(arm_side)
	
	def update(self):
		"""
		Converts RoboPuppet joints to Baxter commands
		:return: None
		"""
		for j in range(num_joints):
			self._joint_angles[self._joint_names[j]] = self._puppet.get_angle(j)
		self._arm.set_joint_positions(self._joint_angles)

"""
Main Function
"""
if __name__ == '__main__':
	node = Arm()
	comm_rate = rospy.get_param('~comm_rate')
	rate = rospy.Rate(comm_rate)
	while not rospy.is_shutdown():
		node.update()
		rate.sleep()

