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
from serial_server import SerialServer
from slew_limiter import SlewLimiter
from clamp_limiter import ClampLimiter
from struct import pack, unpack

"""
Communication Constants
"""
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200
START_BYTE = 0xA5
MSG_HB_ID = 0x01
MSG_HB_RX_LEN = 0
MSG_OM_ID = 0x02
MSG_OM_TX_LEN = 1
MSG_EA_ID = 0x03
MSG_EA_TX_LEN = 2
MSG_JS_ID = 0x04
MSG_JS_RX_LEN = 19
MSG_JZ_ID = 0x05
MSG_JZ_TX_LEN = 2
MSG_VL_ID = 0x06
MSG_VL_TX_LEN = 10
MSG_PG_ID = 0x07
MSG_PG_TX_LEN = 14
UPDATE_RATE_HZ = 10.0

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
		:param port: Serial port name [String]
		:param baud: Baud rate [int]
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
		
		# Serial communication
		self._serial = Serial(port=port, baudrate=baud, timeout=1.0)
		self._server = SerialServer(self._serial, start_byte=START_BYTE)
		self._server.add_rx(MSG_HB_ID, MSG_HB_RX_LEN, self._msg_hb_rx)
		self._server.add_tx(MSG_OM_ID, MSG_OM_TX_LEN, self._msg_om_tx)
		self._server.add_tx(MSG_EA_ID, MSG_EA_TX_LEN, self._msg_ea_tx)
		self._server.add_rx(MSG_JS_ID, MSG_JS_RX_LEN, self._msg_js_rx)
		self._server.add_tx(MSG_JZ_ID, MSG_JZ_TX_LEN, self._msg_jz_tx)
		self._server.add_tx(MSG_VL_ID, MSG_VL_TX_LEN, self._msg_vl_tx)
		self._server.add_tx(MSG_PG_ID, MSG_PG_TX_LEN, self._msg_pg_tx)
		
		# Serial message variables
		self._tx_mode = 'limp'
		self._tx_arm = 'L'
		self._tx_enable = True
		self._tx_joint = 0
		self._tx_kp = 0.0
		self._tx_ki = 0.0
		self._tx_kd = 0.0
		self._tx_v_min = 0.0
		self._tx_v_max = 0.0
		
	def update(self):
		"""
		Gets state data from RoboPuppet and sends commands to Baxter
		"""
		
		# Process RX message
		self._server.rx()
		
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
	
	def _msg_hb_rx(self, rx_data):
		"""
		Heartbeat RX callback
		:param rx_data: Message RX data (none)
		"""
		print('Heartbeat!')
	
	def _msg_om_tx(self, tx_data):
		"""
		Opmode TX callback
		:param tx_data: Message TX data to pack
		
		Message data:
		[00-00]: Mode (0x00 = Limp, 0x01 = Hold)
		"""
		tx_data[0] = (0x00 if self._tx_mode == 'limp' else 0x01)
	
	def _msg_ea_tx(self, tx_data):
		"""
		Enable Arm TX callback
		:param tx_data: Message TX data to pack
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Enable command (0x01 = Enable, 0x00 = Disable)
		"""
		tx_data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		tx_data[1] = (0x01 if self._tx_enable else 0x00)
	
	def _msg_js_rx(self, rx_data):
		"""
		Joint State RX callback
		:param rx_data: Message RX data
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		[02-02]: Calibration status (0x01 = Calibrated, 0x00 = Not)
		[03-06]: Joint angle (float32) [rad]
		[07-10]: Joint velocity (float32) [rad/s]
		[11-14]: Voltage command (float32) [V]
		[15-18]: Motor current (float32) [A]
		"""
		angles = (self._angles_L if rx_data[0] == 0x00 else self._angles_R)
		names = (self._names_L if rx_data[0] == 0x00 else self._names_R)
		joint = rx_data[1]
		angles[names[joint]] = unpack('f', bytearray(rx_data[3:7]))[0]
		# TODO process other data
		pass
		
	def _msg_jz_tx(self, tx_data):
		"""
		Joint Zero TX callback
		:param tx_data: Message TX data
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		"""
		tx_data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		tx_data[1] = self._tx_joint
	
	def _msg_vl_tx(self, tx_data):
		"""
		Voltage Limit TX callback
		:param tx_data: Message TX data
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		[02-05]: Min voltage command (float32) [V]
		[06-09]: Max voltage command (float32) [V]
		"""
		tx_data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		tx_data[1] = self._tx_joint
		tx_data[2:6] = list(pack('f', self._tx_v_min))
		tx_data[6:10] = list(pack('f', self._tx_v_max))
	
	def _msg_pg_tx(self, tx_data):
		"""
		PID Gain TX callback
		:param tx_data: Message TX data
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		[02-05]: P-gain (float32) [V/rad]
		[06-09]: I-gain (float32) [V/(rad*s)]
		[10-13]: D-gain (float32) [V/(rad/s)]
		"""
		tx_data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		tx_data[1] = self._tx_joint
		tx_data[2:6] = list(pack('f', self._tx_kp))
		tx_data[6:10] = list(pack('f', self._tx_ki))
		tx_data[10:14] = list(pack('f', self._tx_kd))

"""
Main Function
"""
if __name__ == '__main__':
	baxter = RoboPuppet(SERIAL_PORT, SERIAL_BAUD)
	rate = rospy.Rate(UPDATE_RATE_HZ)
	while not rospy.is_shutdown():
		baxter.update()
		rate.sleep()
