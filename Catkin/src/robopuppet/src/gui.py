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
from ttk import Notebook
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from math import pi
from time import time

"""
Constants
"""
UPDATE_RATE = 10.0

"""
Main Tab Class
"""
class MainTab:
	
	def __init__(self, parent, puppet):
		"""
		Creates GUI main tab
		:param parent: Tab parent [Notebook]
		:param puppet: RoboPuppet [Interface]
		"""
		
		# Copy interface pointer
		self._puppet = puppet
		
		# Create frame
		self._fr = Frame(parent)
		self._fr.pack()
		parent.add(self._fr, text='Main')
		
		# Heartbeat GUI
		self._fr_top = self._make_fr_top(self._fr)
		self._fr_hb = self._make_fr_hb(self._fr_top)
		self._lb_hb = self._make_lb_hb(self._fr_hb)
		
		# Opmode GUI
		self._fr_om = self._make_fr_om(self._fr_top)
		self._lb_om = self._make_lb_om(self._fr_om)
		self._sv_om = self._make_rbs_om(self._fr_om)
		
		# Joint State GUI
		self._lb_js = self._make_lb_js(self._fr)
		self._fr_js = self._make_fr_js(self._fr)
		self._lbs_js = self._make_lbs_js(self._fr_js)
		
		# Gripper State GUI
		self._lb_gs = self._make_lb_gs(self._fr)
		self._fr_gs = self._make_fr_gs(self._fr)
		self._lbs_gs = self._make_lbs_gs(self._fr_gs)
	
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
	
	def update(self):
		"""
		Updates GUI labels
		"""
		# TODO Update GUI with interface
		pass

"""
Joint Tab Class
"""
class JointTab:
	
	def __init__(self, parent, puppet, joint):
		"""
		Creates GUI joint tab
		:param parent: Tab parent [Notebook]
		:param puppet: RoboPuppet [Interface]
		:param joint: Joint index [0...6]
		"""
		
		# Save arguments
		self._puppet = puppet
		self._joint = joint
		
		# Create frame
		self._fr = Frame(parent)
		self._fr.pack()
		parent.add(self._fr, text=('Joint %u' % self._joint))
		
		# Joint and voltage plots
		self._make_jp(self._fr)
		self._make_vp(self._fr)
		
		# Config frame
		# TODO
	
	def _make_jp(self, parent):
		"""
		Creates joint angle plot
		:param parent: Parent frame
		"""
		
		# Create plot
		self._fig_jp = Figure(figsize=(10, 4))
		self._axs_jp = self._fig_jp.add_subplot(1, 1, 1)
		self._axs_jp.set_title('Joint Angle')
		self._axs_jp.set_xlabel('Time [s]')
		self._axs_jp.set_ylabel('Angle [rad]')
		self._axs_jp.set_xlim(-5.0, +0.0)
		self._axs_jp.set_ylim(-3.2, +3.2)
		self._axs_jp.grid()
		self._fig_jp.tight_layout()
		
		# Create canvas
		self._cv_jp = FigureCanvasTkAgg(self._fig_jp, master=parent)
		self._cv_jp.get_tk_widget().pack(side='top', fill='both')
		self._cv_jp.draw()
		
		# TODO return
		return None
	
	def _make_vp(self, parent):
		"""
		Creates joint voltage plot
		:param parent: Parent frame
		"""
		
		# Create plot
		self._fig_vp = Figure(figsize=(10, 4))
		self._axs_vp = self._fig_vp.add_subplot(1, 1, 1)
		self._axs_vp.set_title('Joint Voltage')
		self._axs_vp.set_xlabel('Time [s]')
		self._axs_vp.set_ylabel('Voltage [V]')
		self._axs_vp.set_xlim(-5.0, +0.0)
		self._axs_vp.set_ylim(-7.2, +7.2)
		self._axs_vp.grid()
		self._fig_vp.tight_layout()
		
		# Create canvas
		self._cv_vp = FigureCanvasTkAgg(self._fig_vp, master=parent)
		self._cv_vp.get_tk_widget().pack(side='top', fill='both')
		self._cv_vp.draw()
		
		# TODO return
		return None
	
	def update(self):
		"""
		Updates GUI plots and labels
		"""
		# TODO update GUI with interface
		pass

"""
GUI Class
"""
class GUI:

	def __init__(self):
		"""
		Initializes GUI node
		"""
		
		# ROS Inits
		rospy.init_node('gui')
		side = 'L' # rospy.get_param('~side')
		puppet = None # Interface(side)
		
		# Tkinter Root
		self._root = Tk()
		self._root.title('RoboPuppet Arm ' + side)
		self._fr_root = Frame(self._root, width=800, height=800)
		# self._fr_root.pack_propagate(False)
		self._fr_root.pack()
		self._nb = Notebook(self._fr_root)
		self._nb.pack(expand=True, fill='both')
		
		# Tabs
		self._tab_main = MainTab(self._nb, puppet)
		self._tab_joints = []
		for j in range(num_joints):
			self._tab_joints.append(JointTab(self._nb, puppet, j))
		
		# Start Tkinter
		self._nb.enable_traversal()
		self._update_ms = int(1000 / UPDATE_RATE)
		self._root.after(self._update_ms, self._update)
		self._root.mainloop()
	
	def _update(self):
		"""
		Updates GUI with RoboPuppet data
		"""
		
		# Update tabs
		self._tab_main.update()
		for tab in self._tab_joints:
			tab.update()
		
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
