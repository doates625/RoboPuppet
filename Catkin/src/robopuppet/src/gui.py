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
		
		# Tkinter Window
		self._root = Tk()
		self._root.title('RoboPuppet Arm ' + side)
		self._fr_root = Frame(self._root, width=800, height=400)
		self._fr_root.pack_propagate(False)
		self._fr_root.pack()
		
		# Top Frame
		self._fr_top = Frame(self._fr_root, height=50, bg='black')
		self._fr_top.pack_propagate(False)
		self._fr_top.pack(side='top', fill=X)
		
		# Heartbeat Frame
		self._fr_hb = Frame(self._fr_top, bg='blue')
		self._fr_hb.pack_propagate(False)
		self._fr_hb.pack(side='left', expand=True, fill=BOTH)
		
		# Heartbeat Label
		self._lb_hb = Label(self._fr_hb, text=('Last Heartbeat: %.2f' % 0.0))
		self._lb_hb.pack(expand=True)
		
		# Opmode Frame
		self._fr_om = Frame(self._fr_top, bg='red')
		self._fr_om.pack_propagate(False)
		self._fr_om.pack(side='right', expand=True, fill=BOTH)
		
		# Opmode Label
		self._lb_om = Label(self._fr_om, text='Opmode')
		self._lb_om.pack(side='top')
		
		# Opmode Buttons
		self._sv_om = StringVar()
		self._sv_om.set(opmode_names[0])
		self._bt_om = dict()
		for name in opmode_names:
			self._bt_om[name] = Radiobutton(
				self._fr_om, text=name, variable=self._sv_om, value=name, indicatoron=0)
			self._bt_om[name].pack(side='left', expand=True, fill=BOTH)
			
		# Joint State Label
		self._lb_js = Label(self._fr_root, text='Joint States')
		self._lb_js.pack(side='top')
		
		# Joint State Frame
		self._fr_js = Frame(self._fr_root, bg='purple')
		self._fr_js.pack_propagate(False)
		self._fr_js.pack(side='top', expand=True, fill=BOTH)
		
		# Joint State Table Labels
		self._lb_ji = Label(self._fr_js, text='Joint Index')
		self._lb_jc = Label(self._fr_js, text='Calibration')
		self._lb_ja = Label(self._fr_js, text='Angle [rad]')
		self._lb_jv = Label(self._fr_js, text='Voltage [V]')
		self._lb_ji.grid(row=0, column=0, sticky=N+S+E+W)
		self._lb_jc.grid(row=0, column=1, sticky=N+S+E+W)
		self._lb_ja.grid(row=0, column=2, sticky=N+S+E+W)
		self._lb_jv.grid(row=0, column=3, sticky=N+S+E+W)
		for x in range(4):
			Grid.columnconfigure(self._fr_js, x, weight=1)
		
		# Joint State Table Values
		self._lb_jis = []
		self._lb_jcs = []
		self._lb_jas = []
		self._lb_jvs = []
		for j in range(num_joints):
			self._lb_jis.append(Label(self._fr_js, text=('%u' % j)))
			self._lb_jcs.append(Label(self._fr_js, text='False'))
			self._lb_jas.append(Label(self._fr_js, text='0.00'))
			self._lb_jvs.append(Label(self._fr_js, text='0.00'))
			self._lb_jis[j].grid(row=j+1, column=0, sticky=N+S+E+W)
			self._lb_jcs[j].grid(row=j+1, column=1, sticky=N+S+E+W)
			self._lb_jas[j].grid(row=j+1, column=2, sticky=N+S+E+W)
			self._lb_jvs[j].grid(row=j+1, column=3, sticky=N+S+E+W)
		
		# Start Tkinter
		self._update_ms = int(1000 / UPDATE_RATE)
		self._root.after(self._update_ms, self._update)
		self._root.mainloop()
	
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
