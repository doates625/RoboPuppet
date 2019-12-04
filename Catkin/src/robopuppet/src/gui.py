#!/usr/bin/env python

"""
gui.py
Node for RoboPuppet arm control GUI
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
import constants
from constants import opmode_names
from constants import num_joints
from constants import num_grippers
from interface import Interface
from Tkinter import *
from threading import Thread

"""
Constants
"""
UPDATE_RATE = 10.0

"""
Class Definition
"""
class GUI:

	def __init__(self):
		"""
		Initializes GUI node
		"""
		
		# ROS Inits
		rospy.init_node('gui')
		side = 'L' # rospy.get_param('~side')
		# puppet = Interface(side)
		
		# Tkinter Root
		self._root = self._make_root(side)
		self._fr_root = self._make_fr_root(self._root)
		
		# Heartbeat GUI
		self._fr_top = self._make_fr_top(self._fr_root)
		self._fr_hb = self._make_fr_hb(self._fr_top)
		self._lb_hb = self._make_lb_hb(self._fr_hb)
		
		# Opmode GUI
		self._fr_om = self._make_fr_om(self._fr_top)
		self._lb_om = self._make_lb_om(self._fr_om)
		self._sv_om = self._make_rbs_om(self._fr_om)
		
		# Joint State GUI
		self._lb_js = self._make_lb_js(self._fr_root)
		self._fr_js = self._make_fr_js(self._fr_root)
		self._lbs_js = self._make_lbs_js(self._fr_js)
		
		# Gripper State GUI
		self._lb_gs = self._make_lb_gs(self._fr_root)
		self._fr_gs = self._make_fr_gs(self._fr_root)
		self._lbs_gs = self._make_lbs_gs(self._fr_gs)
		
		# Start Tkinter
		self._update_ms = int(1000 / UPDATE_RATE)
		self._root.after(self._update_ms, self._update)
		self._root.mainloop()
	
	def _make_root(self, side):
		"""
		Creates, inits, and returns Tkinter root
		:param side: Puppet arm side ['L', 'R']
		"""
		root = Tk()
		root.title('RoboPuppet Arm ' + side)
		return root
	
	def _make_fr_root(self, parent):
		"""
		Creates and returns root frame
		:param parent: Parent frame (root)
		"""
		frame = Frame(parent, width=600, height=300)
		frame.pack_propagate(False)
		frame.pack()
		return frame
	
	def _make_fr_top(self, parent):
		"""
		Creates and returns top frame
		:param parent: Parent frame
		"""
		frame = Frame(parent, height=50)
		frame.pack_propagate(False)
		frame.pack(side='top', fill=X)
		return frame
	
	def _make_fr_hb(self, parent):
		"""
		Creates and returns heartbeat frame
		:param parent: Parent frame
		"""
		frame = Frame(parent)
		frame.pack_propagate(False)
		frame.pack(side='left', expand=True, fill=BOTH)
		return frame
	
	def _make_lb_hb(self, parent):
		"""
		Creates and returns heartbeat label
		:param parent: Parent frame
		"""
		label = Label(parent, text=('Last Heartbeat: %.2f' % 0.0))
		label.pack(expand=True)
		return label
	
	def _make_fr_om(self, parent):
		"""
		Creates and returns opmode frame
		:param parent: Parent frame
		"""
		frame = Frame(parent)
		frame.pack_propagate(False)
		frame.pack(side='right', expand=True, fill=BOTH)
		return frame
	
	def _make_lb_om(self, parent):
		"""
		Creates and returns opmode label
		:param parent: Parent frame
		"""
		label = Label(parent, text='Opmode')
		label.pack(side='top')
		return label
	
	def _make_rbs_om(self, parent):
		"""
		Creates opmode ratio button array
		:param parent: Parent frame
		:return: Button state [StringVar]
		"""
		opmode = StringVar()
		opmode.set(opmode_names[0])
		self._rbs_om = dict()
		for name in opmode_names:
			self._rbs_om[name] = Radiobutton(
				parent, text=name, variable=opmode, value=name, indicatoron=0)
			self._rbs_om[name].pack(side='left', expand=True, fill=BOTH)
		return opmode
	
	def _make_lb_js(self, parent):
		"""
		Creates and returns joint state label
		:param parent: Parent frame
		"""
		label = Label(parent, text='Joint States')
		label.pack(side='top')
		return label
	
	def _make_fr_js(self, parent):
		"""
		Creates and returns joint state frame
		:param parent: Parent frame
		"""
		frame = Frame(parent)
		frame.pack_propagate(False)
		frame.pack(side='top', fill=X)
		return frame
	
	def _make_lbs_js(self, parent):
		"""
		Creates and returns joint state label nested dictionary
		:param parent: Parent frame
		
		Dictionary structure:
		labels['title'][0] = Index title [string]
		labels['title'][1] = Calibration title [string]
		labels['title'][2] = Angle title [string]
		labels['title'][3] = Voltage title [string]
		labels[0...6]['index'] = Joint index ['0'...'6']
		labels[0...6]['cal'] = Calibration status ['True', 'False']
		labels[0...6]['angle'] = Angle ['%+.2f', rad]
		labels[0...6]['voltage'] = Voltage ['%+.2f', V]
		"""
		
		# Create dictionary
		labels = dict()
		
		# Column Titles
		titles = ['Joint Index', 'Calibration', 'Angle [rad]', 'Voltage [V]']
		labels['title'] = dict()
		for (t, title) in enumerate(titles):
			label = Label(parent, text=titles[t])
			label.grid(row=0, column=t, sticky=N+S+E+W)
			labels['title'][t] = label
			Grid.columnconfigure(parent, t, weight=1)
		
		# Joint States
		keys = ['index', 'cal', 'angle', 'voltage']
		for j in range(num_joints):
			labels[j] = dict()
			labels[j][keys[0]] = Label(parent, text=('%u' % j))
			labels[j][keys[1]] = Label(parent, text='False')
			labels[j][keys[2]] = Label(parent, text=('%+.2f' % 0.00))
			labels[j][keys[3]] = Label(parent, text=('%+.2f' % 0.00))
			for (t, key) in enumerate(keys):
				labels[j][key].grid(row=j+1, column=t, sticky=N+S+E+W)
		
		# Return dictionary
		return labels
	
	def _make_lb_gs(self, parent):
		"""
		Creates and returns gripper state label
		:param parent: Parent frame
		"""
		label = Label(parent, text='Gripper States')
		label.pack(side='top')
		return label
	
	def _make_fr_gs(self, parent):
		"""
		Creates and returns gripper state frame
		:param parent: Parent frame
		"""
		frame = Frame(parent)
		frame.pack_propagate(False)
		frame.pack(side='top', fill=X)
		return frame
	
	def _make_lbs_gs(self, parent):
		"""
		Creates and returns gripper state label nested dictionary
		:param parent: Parent frame
		
		Dictionary structure:
		labels[0...3]['title'] = Title label ['Gripper X']
		labels[0...3]['value'] = Value label ['%+.2f', range 0-1]
		"""
		labels = dict()
		for g in range(num_grippers):
			labels[g] = dict()
			labels[g]['title'] = Label(parent, text=('Gripper %u' % g))
			labels[g]['value'] = Label(parent, text=('%.2f' % 0.00))
			labels[g]['title'].grid(row=0, column=g, sticky=N+S+E+W)
			labels[g]['value'].grid(row=1, column=g, sticky=N+S+E+W)
			Grid.columnconfigure(parent, g, weight=1)
		return labels

	def _update(self):
		"""
		Updates GUI with RoboPuppet data
		"""
		
		# TODO update GUI labels
		
		# Schedule next update or shutdown
		if rospy.is_shutdown():
			self._root.destroy()
		else:
			self._root.after(self._update_ms, self._update)

"""
Main Function
"""
if __name__ == '__main__':
	node = GUI()
