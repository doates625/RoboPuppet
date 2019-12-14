#!/usr/bin/env python

"""
live_plot.py
Class for real-time plotting via matplotlib
Written by Dan Oates (WPI Class of 2020)
"""

import numpy as np
from collections import deque

"""
Class Definition
"""
class LivePlot:
	
	def __init__(self, axes, color, time_dur, update_rate):
		"""
		Initializes live plot
		:param axes: Plot axes [Axes]
		:param color: Plot color [string]
		:param time_dur: Plot time duration [s]
		:param update_rate: Plot update rate [Hz]
		"""
		
		# Copy plot settings
		self._axes = axes
		self._color = color
		self._plot = None
		
		# Plot lists
		update_t = 1.0 / update_rate
		self._plot_t = np.arange(-time_dur, 0.5*update_t, update_t)
		self._plot_y = deque()
		for i in range(len(self._plot_t)):
			self._plot_y.append(np.nan)
	
	def update(self, y, render=True):
		"""
		Updates plot with new value
		:param y: Newest plot value
		:param render: Flag to render plot [Bool]
		"""
		
		# Update y-list
		self._plot_y.popleft()
		self._plot_y.append(y)
		
		# Plot on axes
		if render:
			if self._plot is None:
				self._plot = self._axes.plot(
					self._plot_t,
					self._plot_y,
					color=self._color)[0]
			else:
				self._plot.set_ydata(self._plot_y)

