#!/usr/bin/env python

"""
config.py
Node to manage arm config file
Written by Dan Oates (WPI Class of 2020)

References:
- http://wiki.ros.org/rospy/Overview/Services
- http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
- http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
"""
import rospy
from rospy import Subscriber
from constants import num_joints
from robopuppet.srv import GetConfig, GetConfigResponse
from std_msgs.msg import Float32
from ConfigParser import ConfigParser
from collections import OrderedDict

"""
Class Definition
"""
class Config():

	# Default configs
	_default_configs = OrderedDict([
		('home_angle',		+0.000),
		('angle_min',		-1.571),
		('angle_max',		+1.571),
		('velocity_min',	-0.785),
		('velocity_max',	+0.785),
		('voltage_min',		-7.200),
		('voltage_max',		+7.200),
		('pid_kp',			+0.000),
		('pid_ki',			+0.000),
		('pid_kd',			+0.000),
		('sign_angle',		+1.000),
		('sign_motor',		+1.000),
	])

	def __init__(self):
		"""
		Initializes Config Node
		- Opens or creates config file
		- Creates GetConfig services
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
				for (setting, value) in self._default_configs.items():
					self._parser.set(section, setting, '%+.3f' % value)
			self._parser.write(open(self._file_name, 'w'))
		
		# Subscribe to config topics
		self._subs = dict()
		tn = '/puppet/arm_' + self._arm_side
		for j in range(num_joints):
			tnn = tn + '/joint_%u' % j
			self._subs[j] = dict()
			for name in self._default_configs.keys():
				tnnn = tnn + '/' + name
				self._subs[j][name] = Subscriber(tnnn, Float32, self._msg_config, (j, name))
		
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
		self._parser.set(section, setting, '%+.3f' % msg.data)
		self._parser.write(open(self._file_name, 'w'))

"""
Main Function
"""
if __name__ == '__main__':
	node = Config()
	while not rospy.is_shutdown():
		pass
