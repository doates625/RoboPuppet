#!/usr/bin/env python

"""
serial_interface.py
Serial communication node between ROS and RoboPuppet MCU
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
from rospy import Publisher
from rospy import Subscriber
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Uint8
from std_msgs.msg import Float32
from robopuppet.srv import GetConfig
from constants import num_joints
from constants import num_grippers
from constants import config_names
from serial import Serial
from serial_server import SerialServer
from struct import pack
from struct import unpack

"""
Class Definition
"""
class SerialInterface:
	
	"""
	Server Settings
	"""
	_start_byte = 0xA5
	_msg_id_heartbeat = 0x00
	_msg_id_opmode = 0x10
	_msg_id_joint_state = 0x20
	_msg_id_joint_config = 0x30
	_msg_id_joint_setpoint = 0x31
	_msg_id_gripper = 0x40
	_msg_id_buttons = 0x41
	_config_bytes = {
		'home_angle': 0x00,
		'angle_min': 0x01,
		'angle_max': 0x02,
		'velocity_min': 0x03,
		'velocity_max': 0x04,
		'voltage_min': 0x05,
		'voltage_max': 0x06,
		'pid_kp': 0x07,
		'pid_ki': 0x08,
		'pid_kd': 0x09,
		'sign_angle': 0x0A,
		'sign_motor': 0x0B,
	}
	_opmode_bytes = {
		'limp': 0x00,
		'hold': 0x01,
	}
	_enc_stat_dict = {
		0x00 : 'Working',
		0x01 : 'Disconnected',
		0x02 : 'Uncalibrated',
	}

	def __init__(self):
		"""
		Constructs RoboPuppet serial interface
		"""
		
		# Init ROS node
		rospy.init_node('controller')
		
		# Get params from server
		arm_side = rospy.get_param('~arm_side')
		port_name = rospy.get_param('~port_name')
		baud_rate = rospy.get_param('~baud_rate')
		
		# State Data
		self._got_heartbeat = False
		self._enc_stats = ['Unknown'] * num_joints
		self._angles = [0.0] * num_joints
		self._voltages = [0.0] * num_joints
		self._grippers = [0.0] * num_grippers
		self._user_btn = 0
		
		# Serial Server
		self._serial = Serial(port=port_name, baudrate=baud_rate, timeout=1.0)
		self._server = SerialServer(self._serial, start_byte=self._start_byte)
		self._server.add_rx(self._msg_id_heartbeat, 0, self._msg_rx_heartbeat)
		self._server.add_tx(self._msg_id_opmode, 1, self._msg_tx_opmode)
		self._server.add_rx(self._msg_id_joint_state, 10, self._msg_rx_joint_state)
		self._server.add_tx(self._msg_id_joint_config, 6, self._msg_tx_joint_config)
		self._server.add_tx(self._msg_id_joint_setpoint, 5, self._msg_tx_joint_setpoint)
		self._server.add_rx(self._msg_id_gripper, 5, self._msg_rx_gripper)
		self._server.add_rx(self._msg_id_buttons, 1, self._msg_rx_buttons)
		
		# TX variables
		self._tx_opmode = None
		self._tx_joint = None
		self._tx_setting = None
		self._tx_value = None
		
		# ROS topics
		self._topics = dict()
		tn = '/puppet/arm_' + arm_side
		self._topics['heartbeat'] = Publisher(tn + '/heartbeat', Empty, queue_size=10)
		self._topics['opmode'] = Subscriber(tn + '/opmode', String, self._msg_ros_opmode)
		self._topics['joint'] = dict()
		for j in range(num_joints):
			tnj = tn + '/joint_' + str(j)
			topics = dict()
			topics['enc_stat'] = Publisher(tnj + '/enc_stat', String, queue_size=10)
			topics['angle'] = Publisher(tnj + '/angle', Float32, queue_size=10)
			topics['voltage'] = Publisher(tnj + '/voltage', Float32, queue_size=10)
			topics['setpoint'] = Subscriber(tnj + '/setpoint', Float32, self._msg_ros_setpoint, (j,))
			for name in config_names:
				topics[name] = Subscriber(tnj + '/' + name, Float32, self._msg_ros_config, (j, name))
			self._topics['joint'][j] = topics
		self._topics['gripper'] = dict()
		for g in range(num_grippers):
			tng = tn + '/gripper_' + str(g)
			self._topics['gripper'][g] = Publisher(tng, Float32, queue_size=10)
		self._topics['user_btn'] = Publisher(tn + '/user_btn', Uint8, queue_size=10)
		
		# Load settings from config
		name = 'get_config_' + arm_side
		rospy.wait_for_service(name)
		proxy = rospy.ServiceProxy(name, GetConfig)
		for j in range(num_joints):
			resp = proxy(j)
			self._set_config(j, 'home_angle', resp.home_angle)
			self._set_config(j, 'angle_min', resp.angle_min)
			self._set_config(j, 'angle_max', resp.angle_max)
			self._set_config(j, 'velocity_min', resp.velocity_min)
			self._set_config(j, 'velocity_max', resp.velocity_max)
			self._set_config(j, 'voltage_min', resp.voltage_min)
			self._set_config(j, 'voltage_max', resp.voltage_max)
			self._set_config(j, 'pid_kp', resp.pid_kp)
			self._set_config(j, 'pid_ki', resp.pid_ki)
			self._set_config(j, 'pid_kd', resp.pid_kd)
			self._set_config(j, 'sign_angle', resp.sign_angle)
			self._set_config(j, 'sign_motor', resp.sign_motor)
		
		# Disable gravity comp
		self._set_opmode('limp')
	
	def update(self):
		"""
		Processes all RX messages
		:return: True if new heartbeat was received since last update
		"""
		
		# Process messages
		self._server.rx()
		
		# Pub heartbeat
		if self._got_heartbeat:
			self._topics['heartbeat'].publish(Empty())
			self._got_heartbeat = False
		
		# Pub joint states
		for j in range(num_joints):
			topics = self._topics['joint'][j]
			topics['enc_stat'].publish(String(self._enc_stats[j]))
			topics['angle'].publish(Float32(self._angles[j]))
			topics['voltage'].publish(Float32(self._voltages[j]))
		
		# Pub grippers
		for g in range(num_grippers):
			self._topics['gripper'][g].publish(Float32(self._grippers[g]))
		
		# Pub user button
		self._topics['user_btn'].publish(Uint8(self._user_btn))
	
	def _set_opmode(self, opmode):
		"""
		Sets puppet operating mode
		:param opmode: Opmode [string]
		
		Opmode options:
		'limp' - Motors disabled
		'hold' - Gravity compensation
		"""
		self._tx_opmode = opmode
		self._server.tx(self._msg_id_opmode)
	
	def _set_config(self, joint, setting, value):
		"""
		Sets puppet config value
		:param joint: Joint index [0...6]
		:param setting: Config name [string]
		:param value: Value to set [float]
			
		Config options:
		'home_angle' [rad]
		'angle_min' [rad]
		'angle_max' [rad]
		'velocity_min' [rad/s]
		'velocity_max' [rad/s]
		'voltage_min' [V]
		'voltage_max' [V]
		'pid_kp' [V/rad]
		'pid_ki' [V/(rad*s)]
		'pid_kd' [V/(rad/s)]
		'sign_angle' [+1, -1]
		'sign_motor' [+1, -1]
		"""
		self._tx_joint = joint
		self._tx_setting = setting
		self._tx_value = value
		self._server.tx(self._msg_id_joint_config)
	
	def _msg_ros_opmode(self, msg):
		"""
		Sets puppet opmode from message
		:param msg: Opmode [std_msgs/String]
		:return: None
		"""
		self._set_opmode(msg.data)
	
	def _msg_ros_config(self, msg, args):
		"""
		Sets puppet config value from message
		:param msg: Value [std_msgs/Float32]
		:param args: Setting args
		:return: None
		
		Setting args:
		args[0] = Joint index [0...6]
		args[1] = Config name [string]
		"""
		joint, setting = args
		self._set_config(joint, setting, msg.data)
	
	def _msg_ros_setpoint(self, msg, args):
		"""
		Sets joint angle setpoint
		:param msg: Setpoint [std_msgs/Float32]
		:param args: Callback args
		
		Callback args:
		args[0] = Joint index [0...6]
		"""
		joint, = args
		self._tx_joint = joint
		self._tx_value = msg.data
		self._server.tx(self._msg_id_joint_setpoint)
	
	def _msg_rx_heartbeat(self, data):
		"""
		Heartbeat RX callback
		:param data: Message RX data (none)
		:return: None
		
		Sets internal heartbeat flag
		"""
		self._got_heartbeat = True
	
	def _msg_tx_opmode(self, data):
		"""
		Opmode TX callback
		:param data: Message TX data
		:return: None
		
		Data format:
		[0-0]: Mode (enum)
			   0x00 = Limp
			   0x01 = Hold
		"""
		data[0] = self._opmode_bytes[self._tx_opmode]
	
	def _msg_rx_joint_state(self, data):
		"""
		Joint state RX callback
		:param data: Message RX data
		:return: None
		
		Data format:
		[0-0]: Joint number (0-6)
		[1-1]: Encoder status (enum)
			   0x00 = Working
			   0x01 = Disconnected
			   0x02 = Uncalibrated
		[2-5]: Joint angle (float32) [rad]
		[6-9]: Motor voltage (float32) [V]
		"""
		joint = data[0]
		self._enc_stats[joint] = self._enc_stat_dict[data[1]]
		self._angles[joint] = unpack('f', bytearray(data[2:6]))[0]
		self._voltages[joint] = unpack('f', bytearray(data[6:10]))[0]
	
	def _msg_tx_joint_config(self, data):
		"""
		Joint config TX callback
		:param data: Message TX data
		:return: None
		
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
			   0x07 = PID P-gain [V/rad]
			   0x08 = PID I-gain [V/(rad*s)]
			   0x09 = PID D-gain [V/(rad/s)]
			   0x0A = Angle sign [+1, -1]
			   0x0B = Motor sign [+1, -1]
		[2-5]: Setting (float32)
		"""
		data[0] = self._tx_joint
		data[1] = self._config_bytes[self._tx_setting]
		data[2:6] = [ord(b) for b in pack('f', self._tx_value)]
	
	def _msg_tx_joint_setpoint(self, data):
		"""
		Joint setpoint TX callback
		:param data: Message TX data
		:return: None
		
		Data format:
		[0-0]: Joint number (0-6)
		[1-4]: Joint setpoint [rad]
		"""
		data[0] = self._tx_joint
		data[1:5] = [ord(b) for b in pack('f', self._tx_value)]
	
	def _msg_rx_gripper(self, data):
		"""
		Gripper reading RX callback
		:param data: Message RX data
		:return: None
		
		Data format:
		[0-0]: Gripper number (0-3)
		[1-4]: Normalized reading (float32)
		"""
		index = data[0]
		reading = unpack('f', bytearray(data[1:5]))[0]
		self._grippers[index] = reading
	
	def _msg_rx_buttons(self, data):
		"""
		User button RX callback
		:param data: Message RX data
		:return: None
		
		Data format
		[0-0]: Button ID (1-4, 0 = no press)
		"""
		self._user_btn = data[0]

"""
Main Function
"""
if __name__ == '__main__':
	node = SerialInterface()
	comm_rate = rospy.get_param('~comm_rate')
	rate = rospy.Rate(comm_rate)
	while not rospy.is_shutdown():
		node.update()
		rate.sleep()

