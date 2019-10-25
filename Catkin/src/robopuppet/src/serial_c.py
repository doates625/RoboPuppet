"""
Serial-C
Class for transmitting and receiving standard C data types over serial
Written by Dan Oates (WPI Class of 2020)

Valid c-types for read and write methods:
- Signed integers {int8, int16, int32, int64}
- Unsigned integers {uint8, uint16, uint32, uint64}
- Floating-point {float}
"""

from struct import unpack
from struct import pack

"""
C-Type Dictionary
"""
ctype_to_fmt = {
    'int8': ('b', 1),
    'int16': ('h', 2),
    'int32': ('i', 4),
    'int64': ('q', 8),
    'uint8': ('B', 1),
    'uint16': ('H', 2),
    'uint32': ('I', 4),
    'uint64': ('Q', 8),
    'float': ('f', 4),
}

"""
Class Definition
"""
class SerialC:

    def __init__(self, serial):
        """
        Creates SerialC object
        :param serial: Serial interface pointer
        """
        self._serial = serial

    def get_serial(self):
        """
        Returns internal serial object
        """
        return self._serial

    def read(self, ctype):
    	"""
    	Reads C data type from serial
    	:param ctype: Type string (see header)
    	"""
        fmt, size = ctype_to_fmt[ctype]
        return unpack(fmt, self._serial.read(size))[0]

    def write(self, data, ctype):
    	"""
    	Writes C data type from serial
    	:param data: Value to write
    	:param ctype: Type string (see header)
    	"""
        fmt, _ = ctype_to_fmt[ctype]
        self._serial.write(pack(fmt, data))
