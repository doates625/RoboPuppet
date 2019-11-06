#!/usr/bin/env python

"""
RoboPuppet.py
ROS communication node between Baxter and RoboPuppet
"""

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from std_msgs.msg import Bool
from math import pi
from serial import Serial
from serial_c import SerialC
from slew_limiter import SlewLimiter
from clamp_limiter import ClampLimiter

"""
Communication Constants
"""
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200
UPDATE_RATE_HZ = 10.0
BYTE_START = 0xAA
BYTE_MODE_LIMP = 0x00
BYTE_MODE_HOLD = 0x01
BYTE_MODE_HAPTIC = 0x02

"""
Robot Constants
"""
NUM_JOINTS = 7
NUM_GRIPS = 4
JOINT_ANGLE_MINS = [-pi * 1.0] * NUM_JOINTS
JOINT_ANGLE_MAXS = [+pi * 1.0] * NUM_JOINTS
JOINT_RATE_MINS = [-pi / 4.0] * NUM_JOINTS
JOINT_RATE_MAXS = [+pi / 4.0] * NUM_JOINTS

"""
Class Definition
"""
class RoboPuppet():

	def __init__(self, port, baud):
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
		self._grip_L.open()
		self._grip_R.open()
		
		# Puppet state data
		self._calibrated = False
		self._cal_byte = 0x00
		self._angles_L = dict()
		self._angles_R = dict()
		self._slewers_L = []
		self._slewers_R = []
		self._clampers_L = []
		self._clampers_R = []
		for j in range(NUM_JOINTS):
			self._angles_L[self._names_L[j]] = 0.0
			self._angles_R[self._names_R[j]] = 0.0
			self._slewers_L.append(SlewLimiter(
				JOINT_RATE_MINS[j],
				JOINT_RATE_MAXS[j],
				UPDATE_RATE_HZ))
			self._slewers_R.append(SlewLimiter(
				JOINT_RATE_MINS[j],
				JOINT_RATE_MAXS[j],
				UPDATE_RATE_HZ))
			self._clampers_L.append(ClampLimiter(
				JOINT_ANGLE_MINS[j],
				JOINT_ANGLE_MAXS[j]))
			self._clampers_R.append(ClampLimiter(
				JOINT_ANGLE_MINS[j],
				JOINT_ANGLE_MAXS[j]))
		self._grips_L = [0.0] * NUM_GRIPS
		self._grips_R = [0.0] * NUM_GRIPS
		
		# Calibration publishers
		self._pub_cal_L0 = rospy.Publisher('/puppet/calibration/L0', Bool, queue_size=1)
		self._pub_cal_L2 = rospy.Publisher('/puppet/calibration/L2', Bool, queue_size=1)
		self._pub_cal_L4 = rospy.Publisher('/puppet/calibration/L4', Bool, queue_size=1)
		self._pub_cal_L6 = rospy.Publisher('/puppet/calibration/L6', Bool, queue_size=1)
		self._pub_cal_R0 = rospy.Publisher('/puppet/calibration/R0', Bool, queue_size=1)
		self._pub_cal_R2 = rospy.Publisher('/puppet/calibration/R2', Bool, queue_size=1)
		self._pub_cal_R4 = rospy.Publisher('/puppet/calibration/R4', Bool, queue_size=1)
		self._pub_cal_R6 = rospy.Publisher('/puppet/calibration/R6', Bool, queue_size=1)
			
		# Serial communication
		self._serial = Serial(port=port, baudrate=baud, timeout=1.0)
		self._serialc = SerialC(self._serial)
		
	def update(self):
		"""
		Gets state data from RoboPuppet and sends commands to Baxter
		"""
		
		# Send operating mode to RoboPuppet
		self._serialc.write(BYTE_START, 'uint8')
		self._serialc.write(BYTE_MODE_LIMP, 'uint8')
		
		# Get state data from RP
		self._cal_byte = self._serialc.read('uint8')
		self._calibrated = (self._cal_byte == 0xFF)
		for j in range(NUM_JOINTS):
			self._angles_L[self._names_L[j]] =\
				self._clampers_L[j].update(
					self._slewers_L[j].update(
						self._serialc.read('float')))
			self._angles_R[self._names_R[j]] =\
				self._clampers_R[j].update(
					self._slewers_R[j].update(
						self._serialc.read('float')))
		for g in range(NUM_GRIPS):
			self._grips_L[g] = self._serialc.read('float')
			self._grips_R[g] = self._serialc.read('float')
		
		# Send commands to baxter
		self._calibrated = True	# TEMPORARY DEBUG
		if self._calibrated:
			self._arm_L.set_joint_positions(self._angles_L)
			self._arm_R.set_joint_positions(self._angles_R)
			if self._grips_L[0] > 0.5:
				self._grip_L.close()
			else:
				self._grip_L.open()
			if self._grips_R[0] > 0.5:
				self._grip_R.close()
			else:
				self._grip_R.open()
		
		# Publish joint state topics
		self._pub_cal_L0.publish(Bool((self._cal_byte >> 0) & 0x01))
		self._pub_cal_L2.publish(Bool((self._cal_byte >> 1) & 0x01))
		self._pub_cal_L4.publish(Bool((self._cal_byte >> 2) & 0x01))
		self._pub_cal_L6.publish(Bool((self._cal_byte >> 3) & 0x01))
		self._pub_cal_R0.publish(Bool((self._cal_byte >> 4) & 0x01))
		self._pub_cal_R2.publish(Bool((self._cal_byte >> 5) & 0x01))
		self._pub_cal_R4.publish(Bool((self._cal_byte >> 6) & 0x01))
		self._pub_cal_R6.publish(Bool((self._cal_byte >> 7) & 0x01))

"""
Main Function
"""
if __name__ == '__main__':
	baxter = RoboPuppet(SERIAL_PORT, SERIAL_BAUD)
	rate = rospy.Rate(UPDATE_RATE_HZ)
	while not rospy.is_shutdown():
		baxter.update()
		rate.sleep()
