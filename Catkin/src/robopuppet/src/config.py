#!/usr/bin/env python

"""
config.py
Node to manage arm config file
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
from rospy import Subscriber
from constants import num_joints
from constants import config_fmt
from constants import config_defaults
from robopuppet.srv import GetConfig, GetConfigResponse
from std_msgs.msg import Float32
from ConfigParser import ConfigParser

"""
Class Definition
"""
class Config:

	def __init__(self):
		"""
		Initializes Config Node
		- Opens or creates config file
		- Creates GetConfig service
		- Subscribes to config topics
		"""
		
		# Init ROS node
		rospy.init_node('config')
	
		# Get params from server
		self._arm_side = rospy.get_param('~arm_side')
		self._file_name = rospy.get_param('~file_name')
	
		# Attempt to parse config file
		self._parser = ConfigParser()
		read_files = self._parser.read(self._file_name)
		
		# Create file if nonexistent
		if len(read_files) == 0:
			for j in range(num_joints):
				section = 'joint_%u' % j
				self._parser.add_section(section)
				for (setting, value) in config_defaults.items():
					self._parser.set(section, setting, config_fmt % value)
			self._parser.write(open(self._file_name, 'w'))
		
		# Subscribe to config topics
		self._subs = dict()
		tn = '/puppet/arm_' + self._arm_side
		for j in range(num_joints):
			tnn = tn + '/joint_%u' % j
			self._subs[j] = dict()
			for name in config_defaults.keys():
				tnnn = tnn + '/' + name
				self._subs[j][name] = Subscriber(
					tnnn, Float32, self._msg_config, (j, name))
		
		# Create joint config service
		name = 'get_config_' + self._arm_side
		self._service = rospy.Service(name, GetConfig, self._srv_config)
	
	def _srv_config(self, req):
		"""
		Config request service handler
		:param req: GetConfig request
		"""
		section = 'joint_%u' % req.joint
		response = GetConfigResponse(
			self._parser.getfloat(section, 'home_angle'),
			self._parser.getfloat(section, 'angle_min'),
			self._parser.getfloat(section, 'angle_max'),
			self._parser.getfloat(section, 'velocity_min'),
			self._parser.getfloat(section, 'velocity_max'),
			self._parser.getfloat(section, 'voltage_min'),
			self._parser.getfloat(section, 'voltage_max'),
			self._parser.getfloat(section, 'pid_kp'),
			self._parser.getfloat(section, 'pid_ki'),
			self._parser.getfloat(section, 'pid_kd'),
			self._parser.getfloat(section, 'sign_angle'),
			self._parser.getfloat(section, 'sign_motor'))
		return response
	
	def _msg_config(self, msg, args):
		"""
		Updates config fle with new setting
		:param msg: Value to set [std_msgs/Float32]
		:param args: Setting args
		
		Setting args:
		args[0] = Joint index [0...6]
		args[1] = Setting name [string]
		"""
		joint, setting = args
		section = 'joint_%u' % joint
		self._parser.set(section, setting, config_fmt % msg.data)
		self._parser.write(open(self._file_name, 'w'))

"""
Main Function
"""
if __name__ == '__main__':
	node = Config()
	while not rospy.is_shutdown():
		pass

