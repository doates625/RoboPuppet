#!/usr/bin/env python

"""
rpconsts.py
RoboPuppet constants used by multiple files
Written by Dan Oates (WPI Class of 2020)
"""

from math import pi
from collections import OrderedDict

"""
Robot Constants
"""
num_joints = 7
num_grippers = 4
comm_rate = 10.0

"""
Opmode Settings
"""
opmodes = OrderedDict([
	('limp', 0x00),
	('hold', 0x01),
])

"""
Config Settings
"""
configs = OrderedDict([
	('home_angle',		{ 'byte': 0x00, 'default': +0.000 }),
	('angle_min',		{ 'byte': 0x01, 'default': -1.571 }),
	('angle_max',		{ 'byte': 0x02, 'default': +1.571 }),
	('velocity_min',	{ 'byte': 0x03, 'default': -0.785 }),
	('velocity_max',	{ 'byte': 0x04, 'default': +0.785 }),
	('voltage_min',		{ 'byte': 0x05, 'default': -7.200 }),
	('voltage_max',		{ 'byte': 0x06, 'default': +7.200 }),
	('pid_kp',			{ 'byte': 0x07, 'default': +0.000 }),
	('pid_ki',			{ 'byte': 0x08, 'default': +0.000 }),
	('pid_kd',			{ 'byte': 0x09, 'default': +0.000 }),
])
