#!/usr/bin/env python

"""
constants.py
Package-wide RoboPuppet constants
Written by Dan Oates (WPI Class of 2020)
"""

from collections import OrderedDict

# Numerical Constants
num_joints = 7
num_grippers = 4
motor_vcc = 7.2

# Config Params
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
config_defaults = OrderedDict([
	('home_angle',		+0.000),
	('angle_min',		-1.571),
	('angle_max',		+1.571),
	('velocity_min',	-6.283),
	('velocity_max',	+6.283),
	('voltage_min',		-7.200),
	('voltage_max',		+7.200),
	('pid_kp',			+0.000),
	('pid_ki',			+0.000),
	('pid_kd',			+0.000),
	('sign_angle',		+1.000),
	('sign_motor',		+1.000),
])
config_fmt = '%+.3f'

# Opmode Names
opmode_names = [
	'limp',
	'hold',
]

