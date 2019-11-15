"""
Slew Limiter
Class for limiting rates of change of a signal
Written by Dan Oates (WPI Class of 2020)
"""

from clamp_limiter import ClampLimiter

class SlewLimiter:

	def __init__(self, rate_min, rate_max, f_sample):
		"""
		Constructs slew limiter
		:param rate_min: Minimum rate of change (negative number)
		:param rate_max: Maximum rate of change (positive number)
		:param f_sample: Sample frequency
		"""
		self._t_sample = 1.0 / f_sample
		self._delta_clamper = ClampLimiter(0.0, 0.0)
		self.set_min(rate_min)
		self.set_max(rate_max)
		self._val_prev = 0.0
		self._first_frame = True
	
	def set_min(self, rate_min):
		"""
		Updates minimum rate of change
		:param rate_min: Minimum rate (negative number)
		"""
		delta_min = rate_min * self._t_sample
		self._delta_clamper.set_min(delta_min)
	
	def set_max(self, rate_max):
		"""
		Updates maximum rate of change
		:param rate_max: Maximum rate (positive number)
		"""
		delta_max = rate_max * self._t_sample
		self._delta_clamper.set_max(delta_max)
	
	def update(self, val):
		"""
		Returns val slew-limited
		:param val: Value to slew-limit
		
		On first frame, input is assigned directly to output
		"""
		if self._first_frame:
			val_slew = val
			self._first_frame = False
		else:
			delta = self._delta_clamper.update(val - self._val_prev)
			val_slew = self._val_prev + delta
		self._val_prev = val_slew
		return val_slew
	
	def reset(self):
		"""
		Resets slew limiter so that next input assigns directly to output
		"""
		self._first_frame = True
