#!/usr/bin/env python

"""
main_tab.py
Class for RoboPuppet GUI main tab
Written by Dan Oates (WPI Class of 2020)
"""

from Tkinter import Frame
from Tkinter import Label
from Tkinter import StringVar
from Tkinter import Radiobutton
from Tkinter import Grid
from constants import num_joints
from constants import num_grippers
from constants import opmode_names

"""
Class Definition
"""
class MainTab:
	
	def __init__(self, parent, puppet):
		"""
		Creates GUI main tab
		:param parent: Tab parent [Notebook]
		:param puppet: RoboPuppet [Interface]
		"""
		
		# Copy arguments
		self._parent = parent
		self._puppet = puppet
		
		# Create frame
		self._fr = Frame(parent)
		self._fr.pack()
		self._tab_name = 'Main'
		parent.add(self._fr, text=self._tab_name)
		
		# Heartbeat GUI
		self._make_fr_top(self._fr)
		self._make_fr_hb(self._fr_top)
		self._make_lb_hb(self._fr_hb)
		
		# Opmode GUI
		self._make_fr_om(self._fr_top)
		self._make_lb_om(self._fr_om)
		self._make_rbs_om(self._fr_om)
		
		# Joint State GUI
		self._make_lb_js(self._fr)
		self._make_fr_js(self._fr)
		self._make_lbs_js(self._fr_js)
		
		# Gripper State GUI
		self._make_lb_gs(self._fr)
		self._make_fr_gs(self._fr)
		self._make_lbs_gs(self._fr_gs)
	
	def _make_fr_top(self, parent):
		"""
		Creates top frame
		:param parent: Parent frame
		:return: None
		"""
		self._fr_top = Frame(parent, height=50)
		self._fr_top.pack_propagate(False)
		self._fr_top.pack(side='top', fill='x')
	
	def _make_fr_hb(self, parent):
		"""
		Creates heartbeat frame
		:param parent: Parent frame
		:return: None
		"""
		self._fr_hb = Frame(parent)
		self._fr_hb.pack_propagate(False)
		self._fr_hb.pack(side='left', expand=True, fill='both')
	
	def _make_lb_hb(self, parent):
		"""
		Creates heartbeat label
		:param parent: Parent frame
		:return: None
		"""
		self._lb_hb = Label(parent, text=('Last Heartbeat: %.2f' % 0.0))
		self._lb_hb.pack(expand=True)
	
	def _make_fr_om(self, parent):
		"""
		Creates opmode frame
		:param parent: Parent frame
		:return: None
		"""
		self._fr_om = Frame(parent)
		self._fr_om.pack_propagate(False)
		self._fr_om.pack(side='right', expand=True, fill='both')
	
	def _make_lb_om(self, parent):
		"""
		Creates opmode label
		:param parent: Parent frame
		:return: None
		"""
		self._lb_om = Label(parent, text='Opmode')
		self._lb_om.pack(side='top')
	
	def _make_rbs_om(self, parent):
		"""
		Creates opmode ratio button array
		:param parent: Parent frame
		:return: Button state [StringVar]
		"""
		self._sv_om = StringVar()
		self._sv_om.set(opmode_names[0])
		self._rbs_om = dict()
		for name in opmode_names:
			self._rbs_om[name] = Radiobutton(
				parent, text=name, variable=self._sv_om, value=name, indicatoron=0)
			self._rbs_om[name].pack(side='left', expand=True, fill='both')
	
	def _make_lb_js(self, parent):
		"""
		Creates joint state label
		:param parent: Parent frame
		:return: None
		"""
		self._lb_js = Label(parent, text='Joint States')
		self._lb_js.pack(side='top')
	
	def _make_fr_js(self, parent):
		"""
		Creates joint state frame
		:param parent: Parent frame
		:return: None
		"""
		self._fr_js = Frame(parent)
		self._fr_js.pack_propagate(False)
		self._fr_js.pack(side='top', fill='x')
	
	def _make_lbs_js(self, parent):
		"""
		Creates joint state label nested dictionary
		:param parent: Parent frame
		:return: None
		
		Dictionary structure:
		labels['title'][0] = Index title [string]
		labels['title'][1] = Encoder status title [string]
		labels['title'][2] = Angle title [string]
		labels['title'][3] = Voltage title [string]
		labels[0...6]['index'] = Joint index ['0'...'6']
		labels[0...6]['enc_stat'] = Encoders status [string]
		labels[0...6]['angle'] = Angle ['%+.2f', rad]
		labels[0...6]['voltage'] = Voltage ['%+.2f', V]
		"""
		
		# Create dictionary
		self._lbs_js = dict()
		
		# Column Titles
		titles = ['Joint Index', 'Encoder', 'Angle [rad]', 'Voltage [V]']
		self._lbs_js['title'] = dict()
		for (t, title) in enumerate(titles):
			label = Label(parent, text=titles[t])
			label.grid(row=0, column=t, sticky='NSEW')
			self._lbs_js['title'][t] = label
			Grid.columnconfigure(parent, t, weight=1)
		
		# Joint States
		keys = ['index', 'enc_stat', 'angle', 'voltage']
		for j in range(num_joints):
			self._lbs_js[j] = dict()
			self._lbs_js[j][keys[0]] = Label(parent, text=('%u' % j))
			self._lbs_js[j][keys[1]] = Label(parent, text='Unknown')
			self._lbs_js[j][keys[2]] = Label(parent, text=('%+.2f' % 0.00))
			self._lbs_js[j][keys[3]] = Label(parent, text=('%+.2f' % 0.00))
			for (t, key) in enumerate(keys):
				self._lbs_js[j][key].grid(row=j+1, column=t, sticky='NSEW')
	
	def _make_lb_gs(self, parent):
		"""
		Creates and returns gripper state label
		:param parent: Parent frame
		:return: None
		"""
		self._lb_gs = Label(parent, text='Gripper States')
		self._lb_gs.pack(side='top')
	
	def _make_fr_gs(self, parent):
		"""
		Creates and returns gripper state frame
		:param parent: Parent frame
		:return: None
		"""
		self._fr_gs = Frame(parent)
		self._fr_gs.pack_propagate(False)
		self._fr_gs.pack(side='top', fill='x')
	
	def _make_lbs_gs(self, parent):
		"""
		Creates and returns gripper state label nested dictionary
		:param parent: Parent frame
		:return: None
		
		Dictionary structure:
		labels[0...3]['title'] = Title label ['Gripper X']
		labels[0...3]['value'] = Value label ['%+.2f', range 0-1]
		"""
		self._lbs_gs = dict()
		for g in range(num_grippers):
			self._lbs_gs[g] = dict()
			self._lbs_gs[g]['title'] = Label(parent, text=('Gripper %u' % g))
			self._lbs_gs[g]['value'] = Label(parent, text=('%.2f' % 0.00))
			self._lbs_gs[g]['title'].grid(row=0, column=g, sticky='NSEW')
			self._lbs_gs[g]['value'].grid(row=1, column=g, sticky='NSEW')
			Grid.columnconfigure(parent, g, weight=1)
	
	def update(self):
		"""
		Updates GUI labels if tab is selected
		:return: None
		"""
		if self._parent.tab(self._parent.select(), 'text') == self._tab_name:
		
			# Update last heartbeat time
			last_hb = self._puppet.get_last_heartbeat()
			self._lb_hb.config(text=('Last Heartbeat: %.2f' % last_hb))
		
			# Update joint angles
			for j in range(num_joints):
				enc_stat = self._puppet.get_enc_stat(j)
				angle = self._puppet.get_angle(j)
				voltage = self._puppet.get_voltage(j)
				self._lbs_js[j]['enc_stat'].config(text=enc_stat)
				self._lbs_js[j]['angle'].config(text=('%+.2f' % angle))
				self._lbs_js[j]['voltage'].config(text=('%+.2f' % voltage))
		
			# Update gripper readings
			for g in range(num_grippers):
				reading = self._puppet.get_gripper(g)
				self._lbs_gs[g]['value'].config(text=('%.2f' % reading))
			
			# Synchronize mode toggle
			opmode = self._puppet.get_opmode()
			self._sv_om.set(opmode)
