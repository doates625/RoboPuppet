#!/usr/bin/env python

"""
joint_tab.py
Class for RoboPuppet GUI joint tabs
Written by Dan Oates (WPI Class of 2020)
"""

from Tkinter import Frame
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_tkagg import NavigationToolbar2TkAgg
from matplotlib.figure import Figure

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
	
	def update(self):
		"""
		Updates GUI plots and labels
		"""
		# TODO update GUI with interface
		pass

