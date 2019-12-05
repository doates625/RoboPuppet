#!/usr/bin/env python

"""
gui.py
Node for RoboPuppet arm control GUI
Written by Dan Oates (WPI Class of 2020)
"""

import rospy
from constants import num_joints
from interface import Interface
from Tkinter import Tk
from Tkinter import Frame
from ttk import Notebook
from main_tab import MainTab
from joint_tab import JointTab

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
		puppet = Interface(side)
		
		# Tkinter Root
		self._root = Tk()
		self._root.title('RoboPuppet Arm ' + side)
		self._fr_root = Frame(self._root, width=800, height=800)
		self._fr_root.pack()
		self._nb = Notebook(self._fr_root)
		self._nb.pack(fill='both', expand=True)
		
		# GUI Tabs
		self._tab_main = MainTab(self._nb, puppet)
		self._tab_joints = []
		for j in range(num_joints):
			self._tab_joints.append(JointTab(self._nb, puppet, j))
		
		# Start Tkinter
		self._nb.enable_traversal()
		self._update_ms = int(1000.0 / UPDATE_RATE)
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
