#!/usr/bin/env python

"""
rpnode.py
Communication node between ROS, baxter, and RoboPuppet
"""

import rospy
from rospy import Publisher, Subscriber
from std_msgs.msg import Empty, Bool, String, Float32
import baxter_interface
from baxter_interface import CHECK_VERSION
from rpserial import RoboPuppetSerial
from rpconsts import *

"""
Class Definition
"""
class RoboPuppetNode():

	def __init__(self):
		"""
		Constructs RoboPuppet node
		"""
		
		# Init ROS node
		rospy.init_node('RoboPuppet')
		
		# Get params from server
		side = rospy.get_param('~side')
		port = rospy.get_param('~port')
		baud = rospy.get_param('~baud')
		filename = rospy.get_param('~file')
		
		# Baxter interfaces
		self._enabler = baxter_interface.RobotEnable(CHECK_VERSION)
		self._enabler.enable()
		limb_name = ('left' if side == 'L' else 'right')
		self._arm = baxter_interface.Limb(limb_name)
		self._joint_names = self._arm.joint_names()
		self._gripper = baxter_interface.Gripper(limb_name, CHECK_VERSION)
		self._gripper.open()
		
		# Serial interface
		self._puppet = RoboPuppetSerial(port, baud, filename)
		
		# ROS topics
		self._topics = dict()
		tn = '/puppet/arm_' + side
		self._topics['heartbeat'] = Publisher(tn + '/heartbeat', Empty, queue_size=10)
		self._topics['opmode'] = Subscriber(tn + '/opmode', String, self._topic_opmode)
		self._topics['joint'] = dict()
		for j in range(num_joints):
			tnj = tn + '/joint_' + str(j)
			topics = dict()
			topics['calibrated'] = Publisher(tnj + '/calibrated', Bool, queue_size=10)
			topics['angle'] = Publisher(tnj + '/angle', Float32, queue_size=10)
			topics['voltage'] = Publisher(tnj + '/voltage', Float32, queue_size=10)
			for name in configs.keys():
				topics[name] = Subscriber(tnj + '/' + name, Float32, self._topic_config, (j, name,))
			self._topics['joint'][j] = topics
		self._topics['gripper'] = dict()
		for g in range(num_grippers):
			tng = tn + '/gripper_' + str(g)
			self._topics['gripper'][g] = Publisher(tng, Float32, queue_size=10)
	
	def update(self):
		"""
		Processes all TX and RX messages from RoboPuppet
		"""
		
		# Read message and heartbeat
		if self._puppet.update():
			self._topics['heartbeat'].publish(Empty())
		
		# Control Baxter
		angles = [0.0] * num_joints
		for j in range(num_joints):
			angles[j] = self._puppet.get_angle(j)
			self._arm.set_joint_positions({self._joint_names[j]: angles[j]})
		if self._puppet.get_gripper(0) > 0.5:
			self._gripper.close()
		else:
			self._gripper.open()
			
		# Publish ROS state topics
		for j in range(num_joints):
			topics = self._topics['joint'][j]
			topics['calibrated'].publish(Bool(self._puppet.is_calibrated(j)))
			topics['angle'].publish(Float32(self._puppet.get_angle(j)))
			topics['voltage'].publish(Float32(self._puppet.get_voltage(j)))
		for g in range(num_grippers):
			self._topics['gripper'][g].publish(Float32(self._puppet.get_gripper(g)))
	
	def _topic_opmode(self, msg):
		"""
		Sets puppet opmode from message
		:param msg: Opmode [std_msgs/String]
		"""
		self._puppet.set_opmode(msg.data)
	
	def _topic_config(self, msg, args):
		"""
		Sets puppet config value from message
		:param msg: Value [std_msgs/Float32]
		:param args: Setting args
		
		Setting args:
		args[0] = Joint index [0...6]
		args[1] = Config name [string]
		"""
		joint, setting = args
		self._puppet.set_config(joint, setting, msg.data)

if __name__ == '__main__':
	"""
	Node Launcher Function
	"""
	
	# Create node instance
	node = RoboPuppetNode()
	
	# Loop node updates
	rate = rospy.Rate(comm_rate)
	while not rospy.is_shutdown():
		node.update()
		rate.sleep()

