#!/usr/bin/env python

"""
rpserial.py
Serial communication class between Python and RoboPuppet arm microcontroller
Written by Dan Oates (WPI Class of 2020)
"""

from rpconsts import *
from serial import Serial
from serial_server import SerialServer
from ConfigParser import ConfigParser
from struct import pack, unpack
from time import sleep

"""
Class Definition
"""
class RoboPuppetSerial:
	
	"""
	Server Settings
	"""
	_start_byte = 0xA5
	_msg_id_heartbeat = 0x00
	_msg_id_opmode = 0x10
	_msg_id_joint_state = 0x20
	_msg_id_joint_config = 0x30
	_msg_id_gripper = 0x40

	def __init__(self, port, baud, filename=None):
		"""
		Constructs RoboPuppet serial interface
		:param port: Serial port name [string]
		:param baud: Baud rate [int]
		:param filename: Config file name (.ini) [string]
		"""
		
		# State Data
		self._got_heartbeat = False
		self._cals = [0.0] * num_joints
		self._angles = [0.0] * num_joints
		self._voltages = [0.0] * num_joints
		self._grippers = [0.0] * num_grippers
		
		# Serial Server
		self._serial = Serial(port=port, baudrate=baud, timeout=1.0)
		self._server = SerialServer(self._serial, start_byte=self._start_byte)
		self._server.add_rx(self._msg_id_heartbeat, 0, self._msg_rx_heartbeat)
		self._server.add_tx(self._msg_id_opmode, 1, self._msg_tx_opmode)
		self._server.add_rx(self._msg_id_joint_state, 10, self._msg_rx_joint_state)
		self._server.add_tx(self._msg_id_joint_config, 6, self._msg_tx_joint_config)
		self._server.add_rx(self._msg_id_gripper, 5, self._msg_rx_gripper)
		
		# TX variables
		self._tx_opmode = None
		self._tx_joint = None
		self._tx_setting = None
		self._tx_value = None
		
		# Config parsing
		self._filename = filename
		self._config = ConfigParser()
		if self._filename is not None:
		
			# Parse existing file
			self._config.read(self._filename)			
			for joint in range(num_joints):
				section = 'joint_%u' % joint
				for setting in self._config.options(section):
					value = self._config.getfloat(section, setting)
					self.set_config(joint, setting, value)
		
		else:
			
			# Create new file with defaults
			self._filename = 'arm.ini'
			for joint in range(num_joints):
				section = 'joint_%u' % joint
				self._config.add_section(section)
				for (setting, values) in configs.items():
					self.set_config(joint, setting, values['default'])
		
	
	def update(self):
		"""
		Processes all RX messages
		:return: True if new heartbeat was received since last update
		"""
		self._server.rx()
		got_heartbeat = self._got_heartbeat
		self._got_heartbeat = False
		return got_heartbeat
	
	def set_opmode(self, opmode):
		"""
		Sets puppet operating mode
		:param opmode: Opmode [String]
		
		Opmode options:
		'limp': No motor actuation
		'hold': Gravity compensation
		"""
		self._tx_opmode = opmode
		self._server.tx(self._msg_id_opmode)
	
	def is_calibrated(self, joint):
		"""
		Returns joint calibration status
		:param joint: Joint index [0...6]
		"""
		return self._cals[joint]
	
	def get_angle(self, joint):
		"""
		Returns joint angle [rad]
		:param joint: Joint index [0...6]
		"""
		return self._angles[joint]
	
	def get_voltage(self, joint):
		"""
		Returns joint voltage [V]
		:param joint: Joint index [0...6]
		"""
		return self._voltages[joint]
	
	def get_gripper(self, index):
		"""
		Returns gripper reading [0-1]
		:param index: Gripper index [0...3]
		"""
		return self._grippers[index]
	
	def set_config(self, joint, setting, value):
		"""
		Configures joint setting
		:param joint: Joint index [0...6]
		:param setting: Config option [string]
		:param value: Value to set [float]
		
		Config options:
		'home_angle' [rad]
		'min_angle' [rad]
		'max_angle' [rad]
		'min_velocity' [rad/s]
		'max_velocity' [rad/s]
		'min_voltage' [V]
		'max_voltage' [V]
		'pid_kp' [V/rad]
		'pid_ki' [V/(rad*s)]
		'pid_kd' [V/(rad/s)]
		"""
		
		# Send config to microcontroller
		self._tx_joint = joint
		self._tx_setting = setting
		self._tx_value = value		
		self._server.tx(self._msg_id_joint_config)
		
		# Update config file
		section = 'joint_%u' % joint
		self._config.set(section, setting, '%+.3f' % value)
		self._config.write(open(self._filename, 'w'))
	
	def _msg_rx_heartbeat(self, data):
		"""
		Heartbeat RX callback
		:param data: Message RX data (none)
		
		Sets internal heartbeat flag
		"""
		self._got_heartbeat = True
	
	def _msg_tx_opmode(self, data):
		"""
		Opmode TX callback
		:param data: Message TX data
		
		Data format:
		[0-0]: Mode (enum)
			   0x00 = Limp
			   0x01 = Hold
		"""
		data[0] = self.opmodes[self._tx_opmode]
	
	def _msg_rx_joint_state(self, data):
		"""
		Joint state RX callback
		:param data: Message RX data
		
		Data format:
		[0-0]: Joint number (0-6)
		[1-1]: Calibration status (enum)
			   0x00 = Not calibrated
			   0x01 = Calibrated
		[2-5]: Joint angle (float32) [rad]
		[6-9]: Motor voltage (float32) [V]
		"""
		joint = data[0]
		self._cals[joint] = (data[1] == 0x01)
		self._angles[joint] = unpack('f', bytearray(data[2:6]))[0]
		self._voltages[joint] = unpack('f', bytearray(data[6:10]))[0]
	
	def _msg_tx_joint_config(self, data):
		"""
		Joint config TX callback
		:param data: Message TX data
		
		Data format:
		[0-0]: Joint number (0-6)
		[1-1]: Config ID (enum):
			   0x00 = Joint home angle [rad]
			   0x01 = Min angle [rad]
			   0x02 = Max angle [rad]
			   0x03 = Min velocity [rad]
			   0x04 = Max velocity [rad]
			   0x05 = Min voltage [V]
			   0x06 = Max voltage [V]
		[2-5]: Setting (float32)
		"""
		data[0] = self._tx_joint
		data[1] = configs[self._tx_setting]['byte']
		data[2:6] = [ord(b) for b in pack('f', self._tx_value)]
	
	def _msg_rx_gripper(self, data):
		"""
		Gripper reading RX callback
		:param data: Message RX data
		
		Data format:
		[0-0]: Gripper number (0-3)
		[1-4]: Normalized reading (float32)
		"""
		index = data[0]
		reading = unpack('f', bytearray(data[1:5]))[0]
		self._grippers[index] = reading

"""
Main Function (demo)
"""
if __name__ == '__main__':

	# Init puppet interface
	port = '/dev/ttyACM0'
	baud = 115200
	filename = 'arm.ini'
	puppet = RoboPuppetSerial(port, baud, filename)
	
	# Demo printouts
	print('RoboPuppet Serial Test')
	while True:
	
		# Update puppet
		print('State Update:')
		got_heartbeat = puppet.update()
		
		# Heartbeat status
		print('Heartbeat: ' + str(got_heartbeat))
		
		# Angles
		print('Joint Angles:')
		for j in range(num_joints):
			print('\t%i: %+.2f' % (j, puppet.get_angle(j)))
		
		# Calibrations
		print('Joint Cals:')
		for j in range(num_joints):
			print('\t%i: ' % (j,) + str(puppet.is_calibrated(j)))
			
		# Voltages
		print('Joint Voltages:')
		for j in range(num_joints):
			print('\t%i: %+.2f' % (j, puppet.get_voltage(j)))
		
		# Gripper readings
		print('Grippers Readings:')
		for g in range(num_grippers):
			print('\t%i: %+.2f' % (g, puppet.get_gripper(g)))
		
		# Limit print rate
		sleep(0.1)
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
