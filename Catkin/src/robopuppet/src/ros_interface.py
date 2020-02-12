#!/usr/bin/env python

"""
ros_interface.py
ROS message and service interface for RoboPuppet
"""

import rospy
from rospy import Publisher
from rospy import Subscriber
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from robopuppet.srv import GetConfig
from constants import num_joints
from constants import num_grippers
from constants import config_names
from time import time

"""
Class Definition
"""
class ROSInterface:

	def __init__(self, side):
		"""
		Constructs RoboPuppet Interface
		:param side: Arm side ['L', 'R']
		"""
		
		# State data
		self._opmode = 'limp'
		self._enc_stats = ['Unknown'] * num_joints
		self._angles = [0.0] * num_joints
		self._voltages = [0.0] * num_joints
		self._grippers = [0.0] * num_grippers
		self._user_btn = 0
		self._user_btn_prev = 0
		
		# ROS topics
		self._topics = dict()
		tn = '/puppet/arm_' + side
		self._topics['heartbeat'] = Subscriber(tn + '/heartbeat', Empty, self._msg_heartbeat)
		self._topics['opmode_set'] = Publisher(tn + '/opmode', String, queue_size=10)
		self._topics['opmode_get'] = Subscriber(tn + '/opmode', String, self._msg_opmode)
		self._topics['joint'] = dict()
		for j in range(num_joints):
			tnj = tn + '/joint_' + str(j)
			topics = dict()
			topics['enc_stat'] = Subscriber(tnj + '/enc_stat', String, self._msg_joint, (self._enc_stats, j))
			topics['angle'] = Subscriber(tnj + '/angle', Float32, self._msg_joint, (self._angles, j))
			topics['voltage'] = Subscriber(tnj + '/voltage', Float32, self._msg_joint, (self._voltages, j))
			topics['setpoint'] = Publisher(tnj + '/setpoint', Float32, queue_size=10)
			for name in config_names:
				topics[name] = Publisher(tnj + '/' + name, Float32, queue_size=10)
			self._topics['joint'][j] = topics
		self._topics['gripper'] = dict()
		for g in range(num_grippers):
			tng = tn + '/gripper_' + str(g)
			self._topics['gripper'][g] = Subscriber(tng, Float32, self._msg_gripper, (g,))
		self._topics['user_btn'] = Subscriber(tn + '/user_btn', UInt8, self._msg_button)
		
		# Config service proxy
		name = 'get_config_' + side
		rospy.wait_for_service(name)
		self._proxy = rospy.ServiceProxy(name, GetConfig)
		
		# Heartbeat timer
		self._last_heartbeat_time = float('-inf')
		
	def get_last_heartbeat(self):
		"""
		Returns time since last heartbeat [s]
		"""
		return time() - self._last_heartbeat_time
	
	def get_opmode(self):
		"""
		Gets arm opmode [string]
		"""
		return self._opmode
		
	def set_opmode(self, opmode):
		"""
		Sets arm opmode
		:param opmode: 'limp' or 'hold'
		:return: None
		"""
		self._opmode = opmode
		self._topics['opmode_set'].publish(String(self._opmode))
	
	def get_enc_stat(self, joint):
		"""
		Returns encoder status
		:param joint: Joint index [0...6]
		"""
		return self._enc_stats[joint]
	
	def get_angle(self, joint):
		"""
		Returns joint angle [rad]
		:param joint: Joint index [0...6]
		"""
		return self._angles[joint]
	
	def set_setpoint(self, joint, angle):
		"""
		Sets joint angle setpoint
		:param joint: Joint index [0...6]
		:param angle: Angle setpoint [rad]
		"""
		self._topics['joint'][joint]['setpoint'].publish(Float32(angle))
	
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
	
	def get_user_btn(self):
		"""
		Return user button rising edge reading
		1-4 = Rising edge ID
		None = No rising edge
		"""
		if self._user_btn != self._user_btn_prev:
			btn_edge = self._user_btn
		else:
			btn_edge = None
		self._user_btn_prev = self._user_btn
		return btn_edge
		
	def set_config(self, joint, setting, value):
		"""
		Sets joint config variable
		:param joint: Joint index [0...6]
		:param setting: Setting name [string]
		:param value: Value to set [float]
		:return: None
		
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
		'sign_angle' [+1, -1]
		'sign_motor' [+1, -1]
		"""
		self._topics['joint'][joint][setting].publish(Float32(value))
	
	def get_configs(self, joint):
		"""
		Gets joint config variables
		:param joint: Joint index [0...6]
		:return: Joint config [GetConfig]
		"""
		return self._proxy(joint)
	
	def _msg_heartbeat(self, msg):
		"""
		Updates time since last heartbeat
		:param msg: Empty message [Empty]
		:return: None
		"""
		self._last_heartbeat_time = time()
	
	def _msg_opmode(self, msg):
		"""
		Updates opmode
		:param msg: New opmode [String]
		:return: None
		"""
		self._opmode = msg.data
	
	def _msg_joint(self, msg, args):
		"""
		Updates joint state variable
		:param msg: Contains new state
		:param args: Tuple of storage array and joint index [0...6]
		:return: None
		"""
		array, joint = args
		array[joint] = msg.data
	
	def _msg_gripper(self, msg, args):
		"""
		Updates gripper reading
		:param msg: Gripper reading [0-1] [Float32]
		:param args: Tuple of gripper index [0...3]
		:return: None
		"""
		index, = args
		self._grippers[index] = msg.data

	def _msg_button(self, msg):
		"""
		Updates user button reading
		:param msg: Button ID [1-4, 0 = no press, UInt8]
		:return: None
		"""
		self._user_btn = msg.data if msg.data > 0 else None

