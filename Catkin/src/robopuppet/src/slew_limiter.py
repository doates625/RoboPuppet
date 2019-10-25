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
		t_sample = 1.0 / f_sample
		delta_min = rate_min * t_sample
		delta_max = rate_max * t_sample
		self._delta_clamper = ClampLimiter(delta_min, delta_max)
		self._val_prev = 0.0
		self._first_frame = True
	
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
