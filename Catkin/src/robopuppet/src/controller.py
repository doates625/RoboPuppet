#!/usr/bin/env python

"""
controller.py
RoboPuppet and Baxter arm control node
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
import baxter_interface as baxter
from rospy import Publisher
from rospy import Subscriber
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from robopuppet.srv import GetConfig
from constants import num_joints
from constants import num_grippers
from constants import config_names
from baxter_interface import CHECK_VERSION
from serial_comms import SerialComms

"""
Class Definition
"""
class Controller:

	def __init__(self):
		"""
		Initializes Controller Node
		- Creates Baxter controllers
		- Creates RoboPuppet serial interface
		- Subscribes to ROS command topics
		- Loads robot parameters from config file node
		"""
		
		# Init ROS node
		rospy.init_node('controller')
		
		# Get params from server
		arm_side = rospy.get_param('~arm_side')
		port_name = rospy.get_param('~port_name')
		baud_rate = rospy.get_param('~baud_rate')
		
		# Baxter interfaces
		self._enabler = baxter.RobotEnable(CHECK_VERSION)
		self._enabler.enable()
		limb_name = ('left' if arm_side == 'L' else 'right')
		self._arm = baxter.Limb(limb_name)
		self._joint_names = self._arm.joint_names()
		self._joint_angles = dict()
		for j in range(num_joints):
			self._joint_angles[self._joint_names[j]] = 0.0
		
		# Serial interface
		self._puppet = SerialComms(port_name, baud_rate)
		
		# ROS topics
		self._topics = dict()
		tn = '/puppet/arm_' + arm_side
		self._topics['heartbeat'] = Publisher(tn + '/heartbeat', Empty, queue_size=10)
		self._topics['opmode'] = Subscriber(tn + '/opmode', String, self._msg_opmode)
		self._topics['joint'] = dict()
		for j in range(num_joints):
			tnj = tn + '/joint_' + str(j)
			topics = dict()
			topics['calibrated'] = Publisher(tnj + '/calibrated', Bool, queue_size=10)
			topics['angle'] = Publisher(tnj + '/angle', Float32, queue_size=10)
			topics['voltage'] = Publisher(tnj + '/voltage', Float32, queue_size=10)
			for name in config_names:
				topics[name] = Subscriber(tnj + '/' + name, Float32, self._msg_config, (j, name))
			self._topics['joint'][j] = topics
		self._topics['gripper'] = dict()
		for g in range(num_grippers):
			tng = tn + '/gripper_' + str(g)
			self._topics['gripper'][g] = Publisher(tng, Float32, queue_size=10)
		
		# Load settings from config
		name = 'get_config_' + arm_side
		rospy.wait_for_service(name)
		proxy = rospy.ServiceProxy(name, GetConfig)
		for j in range(num_joints):
			resp = proxy(j)
			self._puppet.set_config(j, 'home_angle', resp.home_angle)
			self._puppet.set_config(j, 'angle_min', resp.angle_min)
			self._puppet.set_config(j, 'angle_max', resp.angle_max)
			self._puppet.set_config(j, 'velocity_min', resp.velocity_min)
			self._puppet.set_config(j, 'velocity_max', resp.velocity_max)
			self._puppet.set_config(j, 'voltage_min', resp.voltage_min)
			self._puppet.set_config(j, 'voltage_max', resp.voltage_max)
			self._puppet.set_config(j, 'pid_kp', resp.pid_kp)
			self._puppet.set_config(j, 'pid_ki', resp.pid_ki)
			self._puppet.set_config(j, 'pid_kd', resp.pid_kd)
			self._puppet.set_config(j, 'sign_angle', resp.sign_angle)
			self._puppet.set_config(j, 'sign_motor', resp.sign_motor)
	
	def update(self):
		"""
		Processes all TX and RX messages from RoboPuppet
		:return: None
		"""
		
		# Read messages and heartbeat
		if self._puppet.update():
			self._topics['heartbeat'].publish(Empty())
		
		# Command Baxter joints
		for j in range(num_joints):
			self._joint_angles[self._joint_names[j]] = self._puppet.get_angle(j)
		self._arm.set_joint_positions(self._joint_angles)
			
		# Publish ROS state topics
		for j in range(num_joints):
			topics = self._topics['joint'][j]
			topics['calibrated'].publish(Bool(self._puppet.is_calibrated(j)))
			topics['angle'].publish(Float32(self._puppet.get_angle(j)))
			topics['voltage'].publish(Float32(self._puppet.get_voltage(j)))
		for g in range(num_grippers):
			self._topics['gripper'][g].publish(Float32(self._puppet.get_gripper(g)))
	
	def _msg_opmode(self, msg):
		"""
		Sets puppet opmode from message
		:param msg: Opmode [std_msgs/String]
		:return: None
		"""
		self._puppet.set_opmode(msg.data)
	
	def _msg_config(self, msg, args):
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
		self._puppet.set_config(joint, setting, msg.data)

"""
Main Function
"""
if __name__ == '__main__':
	node = Controller()
	comm_rate = rospy.get_param('~comm_rate')
	rate = rospy.Rate(comm_rate)
	while not rospy.is_shutdown():
		node.update()
		rate.sleep()

