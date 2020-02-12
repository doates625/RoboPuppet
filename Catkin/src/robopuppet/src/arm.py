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
		- Creates Baxter arm interfaces
		- Creates RoboPuppet ROS interface
		"""
		
		# Init ROS node
		rospy.init_node('arm')
		self._side = rospy.get_param('~arm_side')
		self._state = 'calibrating'
		self._ctrl_L = False
		self._ctrl_R = False
		
		# Enable Baxter
		enabler = baxter.RobotEnable(CHECK_VERSION).enable()
		
		# L arm interface
		self._arm_L = baxter.Limb('left')
		self._joint_names_L = self._arm_L.joint_names()
		self._joint_angles_L = dict()
		for j in range(num_joints):
			self._joint_angles_L[self._joint_names_L[j]] = 0.0
			
		# R arm interface
		self._arm_R = baxter.Limb('right')
		self._joint_names_R = self._arm_R.joint_names()
		self._joint_angles_R = dict()
		for j in range(num_joints):
			self._joint_angles_R[self._joint_names_R[j]] = 0.0
		
		# RoboPuppet ROS interface
		self._puppet = ROSInterface(self._side)
	
	def update(self):
		"""
		Converts RoboPuppet joints to Baxter commands
		:return: None
		"""
		
		# Arm control enable state machines
		btn = self._puppet.get_user_btn()
		if btn == 2:
			self._ctrl_L = not self._ctrl_L
		if btn == 3:
			self._ctrl_R = not self._ctrl_R
		
		# Arm control state machine
		if self._state == 'calibrating':
			
			# Check encoder calibrations
			cal = True
			for j in range(4):	# TODO make num_joints...
				if self._puppet.get_enc_stat(j) != 'Working':
					cal = False
					break
			
			# State transition
			if cal:
				self._state = 'enabled'
			
		elif self._state == 'enabled':
		
			# Hold mode check
			if btn == 1:
				opmode = self._puppet.get_opmode()
				if opmode == 'limp': opmode = 'hold'
				elif opmode == 'hold': opmode = 'limp'
				self._puppet.set_opmode(opmode)
			
			# Get joint angles from RoboPuppet
			for j in range(num_joints):
				angle = self._puppet.get_angle(j)
				angle_L = -angle if self._side == 'R' and j % 2 == 0 else +angle
				angle_R = -angle if self._side == 'L' and j % 2 == 0 else +angle
				self._joint_angles_L[self._joint_names_L[j]] = angle_L
				self._joint_angles_R[self._joint_names_R[j]] = angle_R
			
			# Command L arm
			if self._ctrl_L:
				self._arm_L.set_joint_positions(self._joint_angles_L)
		
			# Command R arm
			if self._ctrl_R:
				self._arm_R.set_joint_positions(self._joint_angles_R)

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

