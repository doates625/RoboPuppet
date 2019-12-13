#!/usr/bin/env python

"""
joint_tab.py
Class for RoboPuppet GUI joint tabs
Written by Dan Oates (WPI Class of 2020)
"""

from Tkinter import Frame
from Tkinter import Label
from Tkinter import Button
from Tkinter import Entry
from Tkinter import StringVar
from Tkinter import Grid
from Tkinter import N, S, E, W
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from constants import motor_vcc
from constants import config_names
from constants import frame_rate
from live_plot import LivePlot
from math import pi

"""
Constants
"""
FIGURE_SIZE = (6, 4)
PLOT_DURATION = 5.0
PLOT_ANGLE_MIN = -pi
PLOT_ANGLE_MAX = +pi
PLOT_VOLTAGE_MIN = -motor_vcc
PLOT_VOLTAGE_MAX = +motor_vcc
PLOT_COLOR_J = 'b'
PLOT_COLOR_V = 'r'

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
		self._parent = parent
		self._puppet = puppet
		self._joint = joint
		
		# Create frame
		self._fr = Frame(parent)
		self._fr.pack()
		self._tab_name = 'Joint %u' % self._joint
		parent.add(self._fr, text=self._tab_name)
		
		# Joint and voltage plots
		self._make_fr_plt(self._fr)
		self._make_jp(self._fr_plt)
		self._make_vp(self._fr_plt)
		
		# Config frame
		self._make_fr_cfg(self._fr)
		self._make_lb_cfg(self._fr_cfg)
		self._make_cfgs(self._fr_cfg)
	
	def _make_fr_plt(self, parent):
		"""
		Creates plots frame for angle and voltage
		:param parent: Parent frame
		"""
		self._fr_plt = Frame(parent)
		self._fr_plt.pack(side='left', fill='both')
	
	def _make_jp(self, parent):
		"""
		Creates joint angle plot
		:param parent: Parent frame
		"""
		
		# Create plot
		self._fig_jp = Figure(figsize=FIGURE_SIZE)
		self._axs_jp = self._fig_jp.add_subplot(1, 1, 1)
		self._axs_jp.set_title('Joint Angle')
		self._axs_jp.set_xlabel('Time [s]')
		self._axs_jp.set_ylabel('Angle [rad]')
		self._axs_jp.set_xlim(-PLOT_DURATION, 0)
		self._axs_jp.set_ylim(PLOT_ANGLE_MIN, PLOT_ANGLE_MAX)
		self._axs_jp.grid()
		self._fig_jp.tight_layout()
		
		# Create live plot
		self._live_plot_jp = LivePlot(
			self._axs_jp,
			PLOT_COLOR_J,
			PLOT_DURATION,
			frame_rate)
		
		# Create canvas
		self._cv_jp = FigureCanvasTkAgg(self._fig_jp, master=parent)
		self._cv_jp.get_tk_widget().pack(side='top', fill='both')
		self._cv_jp.draw()
	
	def _make_vp(self, parent):
		"""
		Creates joint voltage plot
		:param parent: Parent frame
		"""
		
		# Create plot
		self._fig_vp = Figure(figsize=FIGURE_SIZE)
		self._axs_vp = self._fig_vp.add_subplot(1, 1, 1)
		self._axs_vp.set_title('Joint Voltage')
		self._axs_vp.set_xlabel('Time [s]')
		self._axs_vp.set_ylabel('Voltage [V]')
		self._axs_vp.set_xlim(-PLOT_DURATION, 0)
		self._axs_vp.set_ylim(PLOT_VOLTAGE_MIN, PLOT_VOLTAGE_MAX)
		self._axs_vp.grid()
		self._fig_vp.tight_layout()
		
		# Create live plot
		self._live_plot_vp = LivePlot(
			self._axs_vp,
			PLOT_COLOR_V,
			PLOT_DURATION,
			frame_rate)
		
		# Create canvas
		self._cv_vp = FigureCanvasTkAgg(self._fig_vp, master=parent)
		self._cv_vp.get_tk_widget().pack(side='top', fill='both')
		self._cv_vp.draw()
	
	def _make_fr_cfg(self, parent):
		"""
		Creates config params frame
		:param parent: Parent frame
		"""
		self._fr_cfg = Frame(parent)
		self._fr_cfg.pack(side='left', fill='both')
	
	def _make_lb_cfg(self, parent):
		"""
		Creates main config label
		:param parent: Parent frame
		"""
		self._lb_cfg = Label(parent, text='Settings')
		self._lb_cfg.grid(row=0, columnspan=2, sticky=N+S+E+W)
	
	def _make_cfgs(self, parent):
		"""
		Creates config labels and entries
		:param parent: Parent frame
		
		Dictionary structures:		
		cfg_lbs[config_name] = Config label
		cfg_ets[config_name] = Config entry
		cfg_svs[config_name] = Config stringvar of entry text
		"""
		
		# Get button
		self._bt_get = Button(parent, text='Get', command=self._bt_get_cb)
		self._bt_get.grid(row=1, column=0, sticky=N+S+E+W)
		
		# Set button
		self._bt_set = Button(parent, text='Set', command=self._bt_set_cb)
		self._bt_set.grid(row=1, column=1, sticky=N+S+E+W)
		
		# Config labels and fields
		self._cfg_lbs = dict()
		self._cfg_ets = dict()
		self._cfg_svs = dict()
		for (i, name) in enumerate(config_names):
			svar = StringVar()
			label = Label(parent, text=name)
			entry = Entry(parent, textvariable=svar)
			label.grid(row=i+2, column=0, sticky=N+S+E+W)
			entry.grid(row=i+2, column=1, sticky=N+S+E+W)
			self._cfg_lbs[name] = label
			self._cfg_ets[name] = entry
			self._cfg_svs[name] = svar
		
		# Set columns to expand
		for c in range(2):
			Grid.columnconfigure(parent, c, weight=1)
		
		# Populate config text fields
		self._bt_get_cb()
	
	def _bt_get_cb(self):
		"""
		Updates config text fields with current config values
		"""
		configs = self._puppet.get_configs(self._joint)
		self._cfg_svs['home_angle'].set('%+.3f' % configs.home_angle)
		self._cfg_svs['angle_min'].set('%+.3f' % configs.angle_min)
		self._cfg_svs['angle_max'].set('%+.3f' % configs.angle_max)
		self._cfg_svs['velocity_min'].set('%+.3f' % configs.velocity_min)
		self._cfg_svs['velocity_max'].set('%+.3f' % configs.velocity_max)
		self._cfg_svs['voltage_min'].set('%+.3f' % configs.voltage_min)
		self._cfg_svs['voltage_max'].set('%+.3f' % configs.voltage_max)
		self._cfg_svs['pid_kp'].set('%+.3f' % configs.pid_kp)
		self._cfg_svs['pid_ki'].set('%+.3f' % configs.pid_ki)
		self._cfg_svs['pid_kd'].set('%+.3f' % configs.pid_kd)
		self._cfg_svs['sign_angle'].set('%+.3f' % configs.sign_angle)
		self._cfg_svs['sign_motor'].set('%+.3f' % configs.sign_motor)
	
	def _bt_set_cb(self):
		"""
		Updates robopuppet with configs in all text fields
		"""
		for name in config_names:
			svar = self._cfg_svs[name]
			try:
				value = float(svar.get())
				self._puppet.set_config(self._joint, name, value)
			except ValueError:
				pass
		self._bt_get_cb()
	
	def update(self):
		"""
		Updates GUI plots if tab is selected
		"""
		angle = self._puppet.get_angle(self._joint)
		voltage = self._puppet.get_voltage(self._joint)
		render = self._parent.tab(self._parent.select(), 'text') == self._tab_name
		self._live_plot_jp.update(angle, render)
		self._live_plot_vp.update(voltage, render)
		if render:
			self._cv_jp.draw()
			self._cv_vp.draw()
