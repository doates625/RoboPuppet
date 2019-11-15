"""
Clamp Limiter
Class for clamp-limiting (saturating) a signal
Written by Dan Oates (WPI Class of 2020)
"""

class ClampLimiter:

	def __init__(self, val_min, val_max):
		"""
		Constructs clamp limiter
		:param val_min: Minimum output
		:param val_max: Maximum output
		"""
		self.set_min(val_min)
		self.set_max(val_max)
	
	def set_min(self, val_min):
		"""
		Update min output value
		:param val_min: Min value
		"""
		self._val_min = val_min
		
	def set_max(self, val_max):
		"""
		Update max output value
		:param val_max: Max value
		"""
		self._val_max = val_max
		
	def update(self, val):
		"""
		Returns val clamp-limited
		:param val: Value to clamp limit
		"""
		return max(self._val_min, min(val, self._val_max))
