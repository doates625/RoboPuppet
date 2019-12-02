#!/usr/bin/env python

"""
debugger.py
ROS node for printing RoboPuppet arm state data
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
import constants
from interface import Interface
from time import sleep

"""
Main Function
"""
if __name__ == '__main__':

	# Initialize node
	rospy.init_node('debugger')
	side = 'L' # rospy.get_param('~side')
	puppet = Interface(side)
	
	# Proportional control demo
	puppet.set_config(3, 'pid_kp', 3.0)
	puppet.set_opmode('hold')
	
	# Print loop
	while not rospy.is_shutdown():
	
		# Print state
		print('Arm' + side + ' State:')
		print('Last heartbeat [s]: %.2f' % puppet.get_last_heartbeat())
		print('Joints:')
		for j in range(constants.num_joints):
			print('\tJoint %i:' % j)
			print('\t\tCalibrated: ' + str(puppet.is_calibrated(j)))
			print('\t\tAngle [rad]: %+.2f' % puppet.get_angle(j))
			print('\t\tVoltage [V]: %+.2f' % puppet.get_voltage(j))
		print('Grippers:')
		for g in range(constants.num_grippers):
			print('\t Gripper %i: %.2f' % (g, puppet.get_gripper(g)))
		print(' ')
		
		# Limit print rate
		sleep(0.2)
