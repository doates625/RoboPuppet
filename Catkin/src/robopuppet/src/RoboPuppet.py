#!/usr/bin/env python

"""
RoboPuppet.py
ROS communication node between Baxter and RoboPuppet
"""

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from std_msgs.msg import Bool, Float32
from math import pi

UPDATE_RATE_HZ = 10.0

class RoboPuppet():

	def __init__(self):
		"""
		Initializes RoboPuppet Node
		- Enables Baxter
		- Subscribes to MCU messages
		"""
		
		# Init node and enable Baxter
		rospy.init_node('RoboPuppet')
		self._enabler = baxter_interface.RobotEnable(CHECK_VERSION)
		self._enabler.enable()

		# Arm interfaces
		self._arm_L = baxter_interface.Limb('left')
		self._arm_R = baxter_interface.Limb('right')
		self._names_L = self._arm_L.joint_names()
		self._names_R = self._arm_R.joint_names()
		
		# Gripper interfaces
		self._grip_L = baxter_interface.Gripper('left', CHECK_VERSION)
		self._grip_R = baxter_interface.Gripper('right', CHECK_VERSION)
		self.set_grip_L('open')
		self.set_grip_R('open')
		
		# Puppet state data
		self._calibrated = False
		self._angles_L = dict()
		self._angles_R = dict()
		for j in range(7):
			self._angles_L[self._names_L[j]] = 0.0
			self._angles_R[self._names_R[j]] = 0.0
		
		# Puppet topic subscriptions
		rospy.Subscriber('/puppet/calibrated', Bool, self._cb_calibrated)
		rospy.Subscriber('/puppet/angles/L0', Float32, self._cb_angles, (self.set_joint_L, 0))
		rospy.Subscriber('/puppet/angles/L1', Float32, self._cb_angles, (self.set_joint_L, 1))
		rospy.Subscriber('/puppet/angles/L2', Float32, self._cb_angles, (self.set_joint_L, 2))
		rospy.Subscriber('/puppet/angles/L3', Float32, self._cb_angles, (self.set_joint_L, 3))
		rospy.Subscriber('/puppet/angles/L4', Float32, self._cb_angles, (self.set_joint_L, 4))
		rospy.Subscriber('/puppet/angles/L5', Float32, self._cb_angles, (self.set_joint_L, 5))
		rospy.Subscriber('/puppet/angles/L6', Float32, self._cb_angles, (self.set_joint_L, 6))
		rospy.Subscriber('/puppet/angles/R0', Float32, self._cb_angles, (self.set_joint_R, 0))
		rospy.Subscriber('/puppet/angles/R1', Float32, self._cb_angles, (self.set_joint_R, 1))
		rospy.Subscriber('/puppet/angles/R2', Float32, self._cb_angles, (self.set_joint_R, 2))
		rospy.Subscriber('/puppet/angles/R3', Float32, self._cb_angles, (self.set_joint_R, 3))
		rospy.Subscriber('/puppet/angles/R4', Float32, self._cb_angles, (self.set_joint_R, 4))
		rospy.Subscriber('/puppet/angles/R5', Float32, self._cb_angles, (self.set_joint_R, 5))
		rospy.Subscriber('/puppet/angles/R6', Float32, self._cb_angles, (self.set_joint_R, 6))
		rospy.Subscriber('/puppet/grippers/L0', Float32, self._cb_grippers, (self.set_grip_L, 0))
		rospy.Subscriber('/puppet/grippers/L1', Float32, self._cb_grippers, (self.set_grip_L, 1))
		rospy.Subscriber('/puppet/grippers/L2', Float32, self._cb_grippers, (self.set_grip_L, 2))
		rospy.Subscriber('/puppet/grippers/L3', Float32, self._cb_grippers, (self.set_grip_L, 3))
		rospy.Subscriber('/puppet/grippers/R0', Float32, self._cb_grippers, (self.set_grip_R, 0))
		rospy.Subscriber('/puppet/grippers/R1', Float32, self._cb_grippers, (self.set_grip_R, 1))
		rospy.Subscriber('/puppet/grippers/R2', Float32, self._cb_grippers, (self.set_grip_R, 2))
		rospy.Subscriber('/puppet/grippers/R3', Float32, self._cb_grippers, (self.set_grip_R, 3))
	
	def update_arms(self):
		"""
		Sends most recent puppet arm angle commands to Baxter
		"""
		self._arm_L.set_joint_positions(self._angles_L)
		self._arm_R.set_joint_positions(self._angles_R)
	
	def get_joint_L(self, index):
		"""
		Reads left arm joint angle
		:param index: Joint index [0-6]
		:return: Joint angle [rad]
		"""
		return self._arm_L.joint_angle(self._names_L[index])
		
	def get_joint_R(self, index):
		"""
		Reads right arm joint angle
		:param index: Joint index [0-6]
		:return: Joint angle [rad]
		"""
		return self._arm_R.joint_angle(self._names_R[index])
	
	def set_joint_L(self, index, angle):
		"""
		Sets left arm joint angle
		:param index: Joint index [0-6]
		:param angle: Joint angle [rad]
		"""
		self._angles_L[self._names_L[index]] = angle
		
	def set_joint_R(self, index, angle):
		"""
		Sets right arm joint angle
		:param index: Joint index [0-6]
		:param angle: Joint angle [rad]
		"""
		self._angles_R[self._names_R[index]] = angle
		
	def set_grip_L(self, state):
		"""
		Sets left gripper state
		:param state: 'open' or 'close'
		"""
		self._set_grip(self._grip_L, state)
	
	def set_grip_R(self, state):
		"""
		Sets left gripper state
		:param state: 'open' or 'close'
		"""
		self._set_grip(self._grip_R, state)
			
	def _set_grip(self, grip, state):
		"""
		Sets gripper state
		:param grip: Baxter gripper interface
		:param state: 'open' or 'close'
		"""
		if state == 'open':
			grip.open()
		elif state == 'close':
			grip.close()
			
	def _cb_calibrated(self, msg):
		"""
		Copies calibration status from RoboPuppet
		:param msg: Calibration status [std_msgs.Bool]
		"""
		self._calibrated = msg.data
	
	def _cb_angles(self, msg, args):
		"""
		Copies joint angle from RoboPuppet
		:param msg: Joint angle [std_msgs.Float32]
		:param args: Arguments (setter function, index)
		"""
		setter = args[0]
		index = args[1]
		setter(index, msg.data)
		
	def _cb_grippers(self, msg, args):
		"""
		Commands gripper based on RoboPuppet gripper message
		:param msg: Gripper channel state [std_msgs.Float32]
		:param args: Arguments (setter function, index)
		"""
		setter = args[0]
		index = args[1]
		if index == 0:
			if msg.data > 0.5:
				setter('close')
			else:
				setter('open')
	
if __name__ == '__main__':
	baxter = RoboPuppet()
	rate = rospy.Rate(UPDATE_RATE_HZ)
	while not rospy.is_shutdown():
		baxter.update_arms()
		rate.sleep()
