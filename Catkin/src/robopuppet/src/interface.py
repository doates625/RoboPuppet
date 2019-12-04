#!/usr/bin/env python

"""
interface.py
Class for interfacing with RoboPuppet through ROS messages
"""

import constants
from constants import num_joints, num_grippers
from robopuppet.srv import GetConfig
import rospy
from rospy import Publisher, Subscriber
from std_msgs.msg import Empty, Bool, String, Float32
from time import time

"""
Class Definition
"""
class Interface():

	def __init__(self, side):
		"""
		Constructs RoboPuppet Interface
		:param side: Arm side ['L', 'R']
		"""
		
		# State data
		self._cals = [False] * num_joints
		self._angles = [0.0] * num_joints
		self._voltages = [0.0] * num_joints
		self._grippers = [0.0] * num_grippers
		
		# ROS topics
		self._topics = dict()
		tn = '/puppet/arm_' + side
		self._topics['heartbeat'] = Subscriber(tn + '/heartbeat', Empty, self._msg_heartbeat)
		self._topics['opmode'] = Publisher(tn + '/opmode', String, queue_size=10)
		self._topics['joint'] = dict()
		for j in range(num_joints):
			tnj = tn + '/joint_' + str(j)
			topics = dict()
			topics['calibrated'] = Subscriber(tnj + '/calibrated', Bool, self._msg_joint, (self._cals, j))
			topics['angle'] = Subscriber(tnj + '/angle', Float32, self._msg_joint, (self._angles, j))
			topics['voltage'] = Subscriber(tnj + '/voltage', Float32, self._msg_joint, (self._voltages, j))
			for name in constants.config_names:
				topics[name] = Publisher(tnj + '/' + name, Float32, queue_size=10)
			self._topics['joint'][j] = topics
		self._topics['gripper'] = dict()
		for g in range(num_grippers):
			tng = tn + '/gripper_' + str(g)
			self._topics['gripper'][g] = Subscriber(tng, Float32, self._msg_gripper, (g,))
		
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
		
	def set_opmode(self, opmode):
		"""
		Sets arm opmode
		:param opmode: 'limp' or 'hold'
		"""
		self._topics['opmode'].publish(String(opmode))
	
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
		Sets joint config variable
		:param joint: Joint index [0...6]
		:param setting: Setting name [string]
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
		return self._proxy(j)
	
	def _msg_heartbeat(self, msg):
		"""
		Updates time since last heartbeat
		:param msg: Empty message [std_msgs/Empty]
		"""
		self._last_heartbeat_time = time()
	
	def _msg_joint(self, msg, args):
		"""
		Updates joint state variable
		:param msg: Contains new state
		:param args: Tuple of storage array and joint index [0...6]
		"""
		array, joint = args
		array[joint] = msg.data
	
	def _msg_gripper(self, msg, args):
		"""
		Updates gripper reading
		:param msg: Gripper reading [0-1] [std_msgs/Float32]
		:param args: Tuple of gripper index [0...3]
		"""
		index, = args
		self._grippers[index] = msg.data

