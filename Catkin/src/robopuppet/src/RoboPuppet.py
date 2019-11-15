#!/usr/bin/env python

"""
RoboPuppet.py
ROS communication node between Baxter and RoboPuppet
"""

import rospy
import baxter_interface
from rospy import Publisher, Subscriber
from baxter_interface import CHECK_VERSION
from std_msgs.msg import Empty, Bool, String, Float32
from math import pi
from serial import Serial
from serial_server import SerialServer
from slew_limiter import SlewLimiter
from clamp_limiter import ClampLimiter
from struct import pack, unpack
from copy import deepcopy

"""
Robot Constants
"""
NUM_JOINTS = 7
NUM_GRIPPERS = 4
DEFAULT_JOINT_ANGLE_MINS = [-pi * 1.0] * NUM_JOINTS
DEFAULT_JOINT_ANGLE_MAXS = [+pi * 1.0] * NUM_JOINTS
DEFAULT_JOINT_VELOCITY_MINS = [-pi / 4.0] * NUM_JOINTS
DEFAULT_JOINT_VELOCITY_MAXS = [+pi / 4.0] * NUM_JOINTS
UPDATE_RATE_HZ = 10.0

"""
Serial Constants
"""
SERIAL_PORT = '/dev/ttyACM0'
SERIAL_BAUD = 115200

"""
Serial Server
"""
START_BYTE = 0xA5
MSG_ID_HEARTBEAT = 0x00
MSG_ID_OPMODE = 0x10
MSG_ID_ENABLE_ARM = 0x11
MSG_ID_JOINT_STATE = 0x20
MSG_ID_JOINT_ZERO = 0x21
MSG_ID_GRIPPER = 0x22
MSG_ID_VOLTAGE_MIN = 0x30
MSG_ID_VOLTAGE_MAX = 0x31
MSG_ID_PID_KP = 0x40
MSG_ID_PID_KI = 0x41
MSG_ID_PID_KD = 0x42

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
		
		# Init Node and Enable Baxter
		rospy.init_node('RoboPuppet')
		self._enabler = baxter_interface.RobotEnable(CHECK_VERSION)
		self._enabler.enable()
		
		# Baxter Interfaces
		limb_names = {'L': 'left', 'R': 'right'}
		self._arm = dict()
		self._joint_names = dict()
		self._grip = dict()
		for arm in ['L', 'R']:
			self._arm[arm] = baxter_interface.Limb(limb_names[arm])
			self._joint_names[arm] = self._arm[arm].joint_names()
			self._grip[arm] = baxter_interface.Gripper(limb_names[arm], CHECK_VERSION)
			self._grip[arm].open()
		
		# Robot State Data
		self._mode = 'limp'
		self._angles = dict()
		self._angle_limiters = dict()
		self._velocity_limiters = dict()
		for arm in ['L', 'R']:
			self._angles[arm] = dict()
			self._angle_limiters[arm] = []
			self._velocity_limiters[arm] = []
			for j in range(NUM_JOINTS):
				self._angles[arm][self._joint_names[arm][j]] = 0.0
				self._angle_limiters[arm].append(ClampLimiter(
					DEFAULT_JOINT_ANGLE_MINS[j],
					DEFAULT_JOINT_ANGLE_MAXS[j]))
				self._velocity_limiters[arm].append(SlewLimiter(
					DEFAULT_JOINT_VELOCITY_MINS[j],
					DEFAULT_JOINT_VELOCITY_MAXS[j],
					UPDATE_RATE_HZ))
		
		# Serial Server
		self._serial = Serial(port=port, baudrate=baud, timeout=1.0)
		self._server = SerialServer(self._serial, start_byte=START_BYTE)
		self._server.add_rx(MSG_ID_HEARTBEAT, 0, self._serialmsg_rx_heartbeat)
		self._server.add_tx(MSG_ID_OPMODE, 1, self._serialmsg_tx_opmode)
		self._server.add_tx(MSG_ID_ENABLE_ARM, 2, self._serialmsg_tx_enable_arm)
		self._server.add_rx(MSG_ID_JOINT_STATE, 19, self._serialmsg_rx_joint_state)
		self._server.add_tx(MSG_ID_JOINT_ZERO, 2, self._serialmsg_tx_joint_zero)
		self._server.add_rx(MSG_ID_GRIPPER, 6, self._serialmsg_rx_gripper)
		self._server.add_tx(MSG_ID_VOLTAGE_MIN, 6, self._serialmsg_tx_control_setting)
		self._server.add_tx(MSG_ID_VOLTAGE_MAX, 6, self._serialmsg_tx_control_setting)
		self._server.add_tx(MSG_ID_PID_KP, 6, self._serialmsg_tx_control_setting)
		self._server.add_tx(MSG_ID_PID_KI, 6, self._serialmsg_tx_control_setting)
		self._server.add_tx(MSG_ID_PID_KD, 6, self._serialmsg_tx_control_setting)
		
		# Server message variables
		self._tx_mode = 'limp'
		self._tx_arm = 'L'
		self._tx_enable = True
		self._tx_j = 0
		self._tx_float = 0.0
		
		# ROS Topic Dictionary
		self._topics = dict()
		self._topics['heartbeat'] = Publisher('/puppet/heartbeat', Empty, queue_size=1)
		self._topics['opmode'] = Subscriber('/puppet/opmode', String, self._rostopic_opmode)
		for arm in ['L', 'R']:
		
			# Arm topic sub-dictionary
			self._topics[arm] = dict()
			name_arm = '/puppet/arm_' + arm
			
			# Enable topic
			self._topics[arm]['enabled'] = Subscriber(name_arm + '/enabled', Bool, self._rostopic_enabled, (arm,))
			
			# Joints sub-dictionary
			self._topics[arm]['joint'] = dict()
			for j in range(NUM_JOINTS):
				
				# Joint sub-dictionary
				topic_joint = dict()
				name_joint = name_arm + '/joint_' + str(j)
				
				# Joint topics
				topic_joint['calibrated'] = Publisher(name_joint + '/calibrated', Bool, queue_size=1)
				topic_joint['angle'] = Publisher(name_joint + '/angle', Float32, queue_size=1)
				topic_joint['angle_min'] = Subscriber(name_joint + '/angle_min', Float32, self._rostopic_safety, (arm, j, self._angle_limiters, ClampLimiter.set_min))
				topic_joint['angle_max'] = Subscriber(name_joint + '/angle_max', Float32, self._rostopic_safety, (arm, j, self._angle_limiters, ClampLimiter.set_max))
				topic_joint['velocity'] = Publisher(name_joint + '/velocity', Float32, queue_size=1)
				topic_joint['velocity_min'] = Subscriber(name_joint + '/velocity_min', Float32, self._rostopic_safety, (arm, j, self._velocity_limiters, SlewLimiter.set_min))
				topic_joint['velocity_max'] = Subscriber(name_joint + '/velocity_max', Float32, self._rostopic_safety, (arm, j, self._velocity_limiters, SlewLimiter.set_max))
				topic_joint['voltage'] = Publisher(name_joint + '/voltage', Float32, queue_size=1)
				topic_joint['voltage_min'] = Subscriber(name_joint + '/voltage_min', Float32, self._rostopic_control, (arm, j, MSG_ID_VOLTAGE_MIN))
				topic_joint['voltage_max'] = Subscriber(name_joint + '/voltage_max', Float32, self._rostopic_control, (arm, j, MSG_ID_VOLTAGE_MAX))
				topic_joint['angle_kp'] = Subscriber(name_joint + '/angle_kp', Float32, self._rostopic_control, (arm, j, MSG_ID_PID_KP))
				topic_joint['angle_ki'] = Subscriber(name_joint + '/angle_ki', Float32, self._rostopic_control, (arm, j, MSG_ID_PID_KI))
				topic_joint['angle_kd'] = Subscriber(name_joint + '/angle_kd', Float32, self._rostopic_control, (arm, j, MSG_ID_PID_KD))
				topic_joint['current'] = Publisher(name_joint + '/current', Float32, queue_size=1)
				topic_joint['zero'] = Publisher(name_joint + '/zero', Empty, queue_size=1)
				
				# Attach sub-dictionary
				self._topics[arm]['joint'][j] = topic_joint
			
			# Grippers sub-dictionary
			self._topics[arm]['gripper'] = dict()
			topic_gripper_name = name_arm + '/gripper_'
			for g in range(NUM_GRIPPERS):
				self._topics[arm]['gripper'][g] = Publisher(topic_gripper_name + str(g), Float32, queue_size=1)
		
	def update(self):
		"""
		Processes serial messages
		"""
		self._server.rx()
	
	def _serialmsg_rx_heartbeat(self, data):
		"""
		Heartbeat RX callback
		:param data: Message RX data (none)
		"""
		self._topics['heartbeat'].publish(Empty())
	
	def _serialmsg_tx_opmode(self, data):
		"""
		Opmode TX callback
		:param data: Message TX data to pack
		
		Message data:
		[00-00]: Mode (0x00 = Limp, 0x01 = Hold)
		"""
		data[0] = (0x00 if self._tx_mode == 'limp' else 0x01)
	
	def _serialmsg_tx_enable_arm(self, data):
		"""
		Enable Arm TX callback
		:param data: Message TX data to pack
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Enable command (0x01 = Enable, 0x00 = Disable)
		"""
		data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		data[1] = (0x01 if self._tx_enable else 0x00)
	
	def _serialmsg_rx_joint_state(self, data):
		"""
		Joint State RX callback
		:param data: Message RX data
		
		Publishes to joint state topics and commands baxter joint angle
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		[02-02]: Calibration status (0x01 = Calibrated, 0x00 = Not)
		[03-06]: Joint angle (float32) [rad]
		[07-10]: Joint velocity (float32) [rad/s]
		[11-14]: Voltage command (float32) [V]
		[15-18]: Motor current (float32) [A]
		"""
		
		# Parse joint ID
		arm = ('L' if data[0] == 0x00 else 'R')
		j = data[1]
		
		# Parse state data
		calibrated = (data[2] == 0x01)
		angle = unpack('f', bytearray(data[3:7]))[0]
		velocity = unpack('f', bytearray(data[7:11]))[0]
		voltage = unpack('f', bytearray(data[11:15]))[0]
		current = unpack('f', bytearray(data[15:19]))[0]
		
		# Publish ROS topics
		topics = self._topics[arm]['joint'][j]
		topics['calibrated'].publish(Bool(calibrated))
		topics['angle'].publish(Float32(angle))
		topics['velocity'].publish(Float32(velocity))
		topics['voltage'].publish(Float32(voltage))
		topics['current'].publish(Float32(current))
		
		# Process angle and send to baxter
		angle = self._velocity_limiters[arm][j].update(angle)
		angle = self._angle_limiters[arm][j].update(angle)
		angle_dict = {self._joint_names[arm][j]: angle}
		self._arm[arm].set_joint_positions(angle_dict)
		
	def _serialmsg_tx_joint_zero(self, data):
		"""
		Joint Zero TX callback
		:param data: Message TX data
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		"""
		data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		data[1] = self._tx_joint
	
	def _serialmsg_rx_gripper(self, data):
		"""
		Gripper Reading RX callback
		:param data: Message RX data
		
		Publishes to ROS topic and commands baxter grippers
		
		Message data:
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Gripper number (0-3)
		[02-05]: Normalized reading (float32)
		"""
		
		# Parse message
		arm = ('L' if data[0] == 0x00 else 'R')
		g = data[1]
		reading = unpack('f', bytearray(data[2:6]))[0]

		# Control baxter grippers
		if g == 0:
			if reading > 0.5:
				self._grip[arm].open()
			else:
				self._grip[arm].close()
		
		# Publish ROS topic
		self._topics[arm]['gripper'][g].publish(Float32(reading))
	
	def _serialmsg_tx_control_setting(self, data):
		"""
		RoboPuppet control setting callback
		:param data: Message TX data to pack
		
		[00-00]: Arm side (0x00 = L, 0x01 = R)
		[01-01]: Joint number (0-6)
		[02-05]: Setting (float32) [V]
		"""
		data[0] = (0x00 if self._tx_arm == 'L' else 0x01)
		data[1] = self._tx_joint
		data[2:6] = list(pack('f', self._tx_float))
	
	def _rostopic_opmode(self, msg):
		"""
		Callback for /opmode topic
		:param msg: Opmode [String]
		
		Sends opmode command to RoboPuppet
		"""
		self._tx_mode = msg.data
		self._server.tx(MSG_ID_OPMODE)
		
	def _rostopic_enabled(self, msg, args):
		"""
		Callback for /enabled topics
		:param msg: Enable status [Bool]
		:param args: Additional params
		
		Sends arm enable/disable command to RoboPuppet
		args[0] = Arm side ['L', 'R']
		"""
		self._tx_arm, = args
		self._tx_enabled = msg.data
		self._server.tx(MSG_ID_ENABLE_ARM)
	
	def _rostopic_safety(self, msg, args):
		"""
		Callback for safety limit topics
		:param msg: Safet limit [Float32]
		:param args: Additional params
		
		Additional params:
		args[0] = Arm side ['L', 'R']
		args[1] = Joint ID [0...6]
		args[2] = Limiters [self._angle_limiters, self._velocity_limiters]
		args[3] = Setter [set_min, set_max]
		"""
		arm, j, limiters, setter = args
		limiters[arm][j].setter(msg.data)
		
	def _rostopic_control(self, msg, args):
		"""
		Callback for RoboPuppet controller settings
		:param msg: Setting [Float32]
		:param args: Additional params
		
		Sends minimum PID voltage command to RoboPuppet
		args[0] = Arm side ['L', 'R']
		args[1] = Joint id [0...6] 
		args[2] = Serial message ID [int]
		"""
		self._tx_arm, self._tx_j, msg_id = args
		self._tx_float = msg.data
		self._server.tx(msg_id)

"""
Main Function
"""
if __name__ == '__main__':
	baxter = RoboPuppet(SERIAL_PORT, SERIAL_BAUD)
	rate = rospy.Rate(UPDATE_RATE_HZ)
	while not rospy.is_shutdown():
		baxter.update()
		rate.sleep()
