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
		self._val_min = val_min
		self._val_max = val_max
		
	def update(self, val):
		"""
		Returns val clamp-limited
		:param val: Value to clamp limit
		"""
		return max(self._val_min, min(val, self._val_max))
