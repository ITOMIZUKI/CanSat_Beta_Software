import time
import serial
import re
import os

class Im920:

    TERMINATOR = b"\r\n"
    ENCODING = "ascii"

    def __init__(self, serial_port:str, baudrate:int=19200, timeout_read:float=10.0, timeout_write:float=10.0):
        """
        serial_port: port name a IM920 is set on
        baudrate: baudrate of IM920
        timeout_read: a read timeout value (changeable later)
        timeout_write: a write timeout value (changeable later)
        ----------------------------------------------------------------------------
        using this class, a device can communicate with other devices which have IM920s on board.
        a user can send or recieve data by communicating with a IM920 in serial communication.
        """

        # # timeout can be changed later
        self.timeout_read = timeout_read
        self.timeout_write = timeout_write

        self.ser = serial.Serial(serial_port, baudrate=baudrate, timeout=self.timeout_read, write_timeout=self.timeout_write)

        # reset all data in a buffer
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # enable to refer state of a buffer
        self.buf_read = self.ser.in_waiting
        self.buf_write = self.ser.out_waiting

    def _write(self, command:str, data:str):
        self.ser.reset_output_buffer()                      # remove noises

        try:
            data = command + data + Im920.TERMINATOR
            self.ser.write(data.encode(Im920.ENCODING), timeout=self.timeout_write)
            self.ser.flush()                                # wait until all data is written
            return 0
        except:
            return 1

    def _edit_setting(self, command:str, data:str):
        self._write("ENWR", "")
        self._write(command, data)
        return self._write("DSWR", "")

    # id
    def set_id(self, id:str):
        id = hex(int(id))[2:]

        while len(id) == 4:
            id = "0" + id

        return self._edit_setting("SRID", id)

    def clean_id(self):
        return self._edit_setting("ERID", "")

    # node
    def set_node(self, node:str):
        return self._edit_setting("STNN", node)

    # channel
    def set_channel(self, channel:str):
        return self._edit_setting("STCH", channel)

    def send(self, data:str)-> "if done well, 0:int, else 1:int":
        """
        data: data to send, whose size has to be less than 64 bytes.
        ------------------------------------------
        this method sends specified data through IM920.
        returns 0, if the execution is done well, else returns 1.
        """
        return self._write("TXDA", data)   

    def receive(self, raw=True)-> "return received data":
        """
        raw: True or False, depending on circumstances
        ---------------------------------------------
        this method returns recieved data.
        if raw is True, then it returns data of list.
        index 0: node number
        index 1: ID number of IM920 that send data
        index 2: RSSI value
        index 3: data
        it returns only data if raw is False.
        """

        data_list = self._receive2list()

        if raw:
            return data_list
        else:
            return data_list[3]

    def receive_lines(self, num_lines, raw=True)-> "return received data":
        """
        num_lines: number of lines to read
        raw: True or False, depending on circumstances
        ---------------------------------------------
        this method returns recieved data of multi lines.
        if raw is True, then it returns data of list.
        index 0: node number
        index 1: ID number of IM920 that send data
        index 2: RSSI value
        index 3: data
        it returns only data if raw is False.
        """
        
        data_lines = []
        for _ in range(num_lines):
            if self.ser.in_waiting:
                break
            data_lines.append(self.receive(raw=raw))

        return data_lines


    def _receive2list(self):
        data = self.ser.read_until(expected=Im920.TERMINATOR, timeout=self.timeout_read).decode("utf-8")[:-2]    # remove CRLF
        return re.split("[,:]", data)                                                                            # split data and change it into a list


if __name__ == "__main__":
    SERIAL_PORT_IM920 = "/dev/ttyUSB1"
    im920 = Im920(SERIAL_PORT_IM920)
    im920.send("hello")