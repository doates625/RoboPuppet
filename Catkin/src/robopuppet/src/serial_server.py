"""
serial_server
Class for transmitting, receiving, and processing serial data packets
Written by Dan Oates (WPI Class of 2020)
"""

from struct import pack, unpack


class SerialServer:

    def __init__(self, serial, start_byte=0x00):
        """
        Creates serial server
        :param serial: Serial interface
        :param start_byte: Message start byte
        """
        self._serial = serial
        self._start_byte = start_byte
        self._tx_ids = []
        self._rx_ids = []
        self._tx_data_lens = []
        self._rx_data_lens = []
        self._tx_funcs = []
        self._rx_funcs = []
        self._tx_data = []
        self._rx_data = []

    def get_serial(self):
        """
        Returns internal serial interface
        """
        return self._serial

    def add_tx(self, msg_id, data_len, func=None):
        """
        Adds TX message to server
        :param msg_id: Message ID [0-255]
        :param data_len: Byte length of message data [0-255]
        :param func: Function to pack data
        :return: None
        """
        self._tx_ids.append(msg_id)
        self._tx_data_lens.append(data_len)
        self._tx_funcs.append(func)

    def add_rx(self, msg_id, data_len, func=None):
        """
        Adds RX message to server
        :param msg_id: Message ID [0-255]
        :param data_len: Byte length of message data [0-255]
        :param func: Function to unpack data
        :return: None
        """
        self._rx_ids.append(msg_id)
        self._rx_data_lens.append(data_len)
        self._rx_funcs.append(func)

    def tx(self, msg_id):
        """
        Transmits message with given ID
        :param msg_id: ID of message to transmit
        :return: True if ID was valid, false otherwise
        """
        if msg_id is not None:
            try:
                tx_i = self._tx_ids.index(msg_id)
            except ValueError:
                return False
            self._tx_index(tx_i)
            return True

    def tx_all(self):
        """
        Transmits all messages in server once
        :return: None
        """
        for tx_i in range(len(self._tx_ids)):
            self._tx_index(tx_i)

    def _tx_index(self, tx_i):
        """
        Transmits message of given index in server
        :param tx_i: Index of message in lists
        :return: None
        """
        self._tx_data = [0] * self._tx_data_lens[tx_i]
        self._tx_funcs[tx_i](self._tx_data)
        checksum = sum(self._tx_data) % 256
        msg = [self._start_byte, self._tx_ids[tx_i]] + self._tx_data + [checksum]
        self._serial.write(bytearray(msg))

    def rx(self):
        """
        Processes all incoming serial messages
        :return: None
        """
        while self._serial.in_waiting:

            # Check for start byte
            start_byte = unpack('B', self._serial.read())[0]
            if start_byte == self._start_byte:

                # Find message by ID
                msg_id = unpack('B', self._serial.read())[0]
                rx_i = 0
                try:
                    rx_i = self._rx_ids.index(msg_id)
                except ValueError:
                    self._serial.flushInput()
                    return

                # Read message data and calculate checksum
                self._rx_data = []
                for b in range(self._rx_data_lens[rx_i]):
                	self._rx_data.append(unpack('B', self._serial.read())[0])
                checksum = sum(self._rx_data) % 256

                # Process packet if checksum is correct
                if unpack('B', self._serial.read())[0] == checksum:
                    self._rx_funcs[rx_i](self._rx_data)
            else:
                self._serial.flushInput()
                return
