#!/usr/bin/env python

"""
constants.py
Package-wide RoboPuppet constants
Written by Dan Oates (WPI Class of 2020)
"""

num_joints = 7
num_grippers = 4
comm_rate = 10.0
motor_vcc = 7.2
config_names = [
	'home_angle',
	'angle_min',
	'angle_max',
	'velocity_min',
	'velocity_max',
	'voltage_min',
	'voltage_max',
	'pid_kp',
	'pid_ki',
	'pid_kd',
	'sign_angle',
	'sign_motor',
]
opmode_names = [
	'limp',
	'hold',
]
