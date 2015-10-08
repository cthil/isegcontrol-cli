#
# IsegShqDevice.py - ISEG SHQ command line controller
#
# Copyright (c) 2015 Christophe Thil <christophe.thil@ziti.uni-heidelberg.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

# Force Python 3.x behaviour for older 2.x interpreters
from __future__ import print_function # print() function with optional destination
from __future__ import division # Division with / alawys returns a float

import NamedConstant

import time
import re
# Also imports serial or ftdi1, depending on instantiation


"""
IsegShqDevice.py
Remote control software for Iseg SHQ high voltage supplies using plain serial or FTDI compatible USB connectivity.

Christophe Thil <christophe.thil@ziti.uni-heidelberg.de>
"""

class IsegShqDevice:
    OUTPUT_STATUS_ON = NamedConstant.NamedConstant('OUTPUT_STATUS_ON', 0)
    OUTPUT_STATUS_OFF = NamedConstant.NamedConstant('OUTPUT_STATUS_OFF', 1)
    OUTPUT_STATUS_MANUAL = NamedConstant.NamedConstant('OUTPUT_STATUS_MANUAL', 2)
    OUTPUT_STATUS_COMPLIANCE = NamedConstant.NamedConstant('OUTPUT_STATUS_COMPLIANCE', 3)
    OUTPUT_STATUS_INHIBITED = NamedConstant.NamedConstant('OUTPUT_STATUS_INHIBITED', 4)
    OUTPUT_STATUS_BAD_QUALITY = NamedConstant.NamedConstant('OUTPUT_STATUS_BAD_QUALITY', 5)
    OUTPUT_STATUS_RISING = NamedConstant.NamedConstant('OUTPUT_STATUS_RISING', 6)
    OUTPUT_STATUS_FALLING = NamedConstant.NamedConstant('OUTPUT_STATUS_FALLING', 7)
    OUTPUT_STATUS_TRIPPED = NamedConstant.NamedConstant('OUTPUT_STATUS_TRIPPED', 8)

    def __init__(self, serial_device=None, usb_vendor_id=None, usb_product_id=None, read_delay = 0.003):
        """Iseg SHQ HV Control object using plain serial or FTDI USB connectivity.

        Keyword arguments:
        serial_device -- the full device name or number of device (starting at zero) for a plain serial interface
        usb_vendor_id -- the vendor ID for a FTDI compatible USB interface
        usb_product_id -- the product ID for a FTDI compatible USB interface
        read_delay -- the delay (in seconds) between a charater write and its readback attempt. Default is 3ms.

        Note: Either set serial device or usb_vendor_id and usb_product_id, depending if you want plain serial or USB connectivity.

        """
        self._read_delay = read_delay
        if serial_device != None:
            self.__init_serial(serial_device)
        elif usb_vendor_id != None and usb_product_id != None:
            self.__init_usb(usb_vendor_id, usb_product_id)
        else:
            self.__serial = None
            self.__ftdi = None
            raise Exception('Either serial or USB connectivity is required.')

    def __init_serial(self, serial_device):
        import serial
        self.__ftdi = None
        try:
            self.__serial = serial.Serial(port=serial_device, baudrate=9600, bytesize=8, parity='N', stopbits=1, xonxoff=False, rtscts=False, dsrdtr=False)
        except serial.SerialException:
            self.__serial = None
            raise Exception('Could not open serial port.')

    def __init_usb(self, usb_vendor_id, usb_product_id):
        import ftdi1
        self.__serial = None
        self.__ftdi = ftdi1
        self.__ftdi_context = self.__ftdi.new()
        if self.__ftdi.usb_open(self.__ftdi_context, usb_vendor_id, usb_product_id) != 0:
            error_string = self.__ftdi.get_error_string(self.__ftdi_context)
            self.__ftdi = None
            raise Exception('Could not open USB port: {0}'.format(error_string))
        else:
            self.__ftdi.set_bitmode(self.__ftdi_context, self.__ftdi.INTERFACE_A, self.__ftdi.BITMODE_RESET)
            self.__ftdi.set_baudrate(self.__ftdi_context, 9600)
            self.__ftdi.set_line_property(self.__ftdi_context, self.__ftdi.BITS_8, self.__ftdi.STOP_BIT_1, self.__ftdi.NONE)
            self.__ftdi.setflowctrl(self.__ftdi_context, 0)
            self.__ftdi.usb_purge_buffers(self.__ftdi_context)

    def __del__(self):
        if self.__serial != None:
            self.__del_serial()
        elif self.__ftdi != None:
            self.__del_usb()
        else:
            pass


    def __del_serial(self):
        self.__serial.close()

    def __del_usb(self):
        self.__ftdi.free(self.__ftdi_context)

    def _write(self, data):
        """
        Blocking write to the device.

        Keyword arguments:
        data -- the string of data bytes to send to the device

        """
        if self.__serial != None:
            self.__serial.write(data)
        elif self.__ftdi != None:
            send_buffer = data
            while len(send_buffer) > 0:
                sent_length = self.__ftdi.write_data(self.__ftdi_context, send_buffer, len(send_buffer))
                send_buffer = send_buffer[sent_length:]
        else:
            raise Exception('No device opened.')

    def _read(self, size = 1):
        """
        Blocking read of a given length from the device.

        Keyword arguments:
        size -- the number of bytes to read from the device

        """
        if self.__serial != None:
            return self.__serial.read(size)
        elif self.__ftdi != None:
            reassembly_buffer = ''
            while len(reassembly_buffer) < size:
                read_length, receive_buffer = self.__ftdi.read_data(self.__ftdi_context, size)
                if read_length > 0:
                    reassembly_buffer += receive_buffer[0:read_length]
            return reassembly_buffer
        else:
            raise Exception('No device opened.')

    def _test_bit(self, value, position):
        """
        Test if a bit is set in the binary representation of a value.

        Keyword arguments:
        value -- the value to check
        position -- the position to check if set

        """
        mask = 1 << position
        return (value & mask) != 0

    def command(self, command):
        """
        Send a command word character by character to the device. Check reply to be in sync and process response.

        Keyword arguments:
        command -- the command word to send to the device

        """
        _command = '{0}\r\n'.format(command)
        while len(_command) > 0:
            character_to_send = _command[0]
            read_back_character = ''
            _command = _command[1:]
            while character_to_send != read_back_character:
                self._write(character_to_send)
                time.sleep(self._read_delay)
                read_back_character = self._read()
        answer = ''
        while len(answer) < 2 or answer[-2:] != '\r\n':
            time.sleep(self._read_delay)
            #
            # FIXME Why does appending the string with += sometimes leads to a conversion
            #       from \r to \n?
            #
            #answer += self._read()
            answer = ''.join((answer, self._read()))
        return answer[:-2]

    def get_device_id(self):
        """
        Get the device identification.

        A dictionary of the following elements is returned:
        identification_number -- Identification number of the device
        software_version -- Version of the controller software
        maximum_voltage -- Maximum voltage the device is able to generate (in Volts)
        maximum_current -- Maximum current the device is able to generate (in Ampere)

        """
        id_string = self.command('#')
        id_matched = re.match('(\d+);(\d+\.\d+);(\d+)([k]*)V;(\d+)([um])A', id_string)
        if id_matched:
            identification_number = int(id_matched.group(1))
            software_version = float(id_matched.group(2))
            maximum_voltage = int(id_matched.group(3))
            maximum_voltage_exponent = id_matched.group(4)
            if maximum_voltage_exponent == 'k':
                maximum_voltage *= 1e3
            maximum_current = int(id_matched.group(5))
            maximum_current_exponent = id_matched.group(6)
            if maximum_current_exponent == 'u':
                maximum_current *= 1e-6
            elif maximum_current_exponent == 'm':
                maximum_current *= 1e-3
            return { 'identification_number' : identification_number,
                     'software_version' : software_version,
                     'maximum_voltage' : maximum_voltage,
                     'maximum_current' : maximum_current }
        else:
            raise ValueError('Parser error for id_string "{0}"'.format(id_string))

    def get_communication_delay(self):
        """
        Get the communication delay of the device

        Returns the delay in milliseconds.

        """
        delay_string = self.command('W')
        return int(delay_string)

    def set_communication_delay(self, delay):
        """
        Set the communication delay [0..255] of the device (in milliseconds).

        Keyword arguments:
        delay -- Communication delay (in milliseconds)

        """
        if delay >= 0 and delay <= 255:
            _command = 'W={0:03d}'.format(delay)
            self.command(_command)
        else:
            raise ValueError('delay must be an integer between 0 and 255.')

    def get_voltage(self):
        """
        Get the current voltage.

        Returns the voltage in volts.

        """
        voltage_string = self.command('U1')
        voltage_matched = re.match('([+-]?)(\d+)([+-])(\d+)', voltage_string)
        if voltage_matched:
            mantissa_signum = -1 if voltage_matched.group(1) == '-' else +1
            mantissa = int(voltage_matched.group(2))
            exponent_signum = -1 if voltage_matched.group(3) == '-' else +1
            exponent = int(voltage_matched.group(4))
            return mantissa_signum * mantissa * 10**(exponent_signum * exponent)
        else:
            raise ValueError('Parser error for voltage_string "{0}"'.format(voltage_string))

    def get_current(self):
        """
        Get the current current.

        Returns the current in ampere.

        """
        current_string = self.command('I1')
        if current_string[0:8] == 'OVERFLOW':
            return float('inf')
        else:
            current_matched = re.match('(\d+)([+-])(\d+)', current_string)
            if current_matched:
                mantissa = int(current_matched.group(1))
                exponent_signum = -1 if current_matched.group(2) == '-' else +1
                exponent = int(current_matched.group(3))
                return mantissa * 10**(exponent_signum * exponent)
            else:
                raise ValueError('Parser error for current_string "{0}"'.format(current_string))

    def get_voltage_limit(self):
        """
        Get the voltage limit.

        Returns the voltage limit as a fraction [0...1] of the maximum voltage

        """
        voltage_limit_string = self.command('M1')
        voltage_limit = int(voltage_limit_string) / 100.
        return voltage_limit

    def get_current_limit(self):
        """
        Get the current limit.

        Returns the current limit as a fraction [0...1] of the maximum current.

        """
        current_limit_string = self.command('N1')
        current_limit = int(current_limit_string) / 100.
        return current_limit

    def get_voltage_preset(self):
        """
        Get the voltage preset.

        Returns the voltage preset in volts.

        """
        voltage_string = self.command('D1')
        voltage_matched = re.match('([+-]?)(\d+)([+-])(\d+)', voltage_string)
        if voltage_matched:
            mantissa_signum = -1 if voltage_matched.group(1) == '-' else +1
            mantissa = int(voltage_matched.group(2))
            exponent_signum = -1 if voltage_matched.group(3) == '-' else +1
            exponent = int(voltage_matched.group(4))
            return mantissa_signum * mantissa * 10**(exponent_signum * exponent)
        else:
            raise ValueError('Parser error for voltage_string "{0}"'.format(voltage_string))

    def set_voltage_preset(self, voltage):
        """
        Set the voltage preset.

        Keyword arguments:
        voltage -- preset voltage in volts.

        """
        _command = 'D1={0:07.2f}'.format(voltage)
        self.command(_command)

    def get_voltage_ramping_speed(self):
        """
        Get the voltage ramping speed.

        Returns the ramping speed in volt/s.

        """
        ramping_speed_string = self.command('V1')
        ramping_speed = int(ramping_speed_string)
        return ramping_speed

    def set_voltage_ramping_speed(self, ramping_speed):
        """
        Set the voltage ramping speed.

        Keyword arguments:
        ramping_speed -- the raming speed [2-255] in volt/s.

        """
        if ramping_speed >= 2 and ramping_speed <= 255:
            _command = 'V1={0:03d}'.format(ramping_speed)
            self.command(_command)
        else:
            raise ValueError

    def apply_voltage_preset(self):
        """
        Apply the voltage preset with the given ramping speed to the output.

        """
        status_string = self.command('G1')
        if status_string == 'S1=LAS': # Re-read output status
            return self.get_output_status()
        elif status_string == 'S1=ON':
            return IsegShqDevice.OUTPUT_STATUS_ON
        elif status_string == 'S1=OFF':
            return IsegShqDevice.OUTPUT_STATUS_OFF
        elif status_string == 'S1=MAN':
            return IsegShqDevice.OUTPUT_STATUS_MANUAL
        elif status_string == 'S1=ERR':
            return IsegShqDevice.OUTPUT_STATUS_COMPLIANCE
        elif status_string == 'S1=INH':
            return IsegShqDevice.OUTPUT_STATUS_INHIBITED
        elif status_string == 'S1=QUA':
            return IsegShqDevice.OUTPUT_STATUS_BAD_QUALITY
        elif status_string == 'S1=L2H':
            return IsegShqDevice.OUTPUT_STATUS_RISING
        elif status_string == 'S1=H2L':
            return IsegShqDevice.OUTPUT_STATUS_FALLING
        elif status_string == 'S1=TRP':
            return IsegShqDevice.OUTPUT_STATUS_TRIPPED
        else:
            raise ValueError('Parser error for status_string "{0}"'.format(status_string))

    def get_output_status(self):
        """
        Read the output status word and decode it.

        """
        status_string = self.command('S1')
        if status_string == 'S1=ON':
            return IsegShqDevice.OUTPUT_STATUS_ON
        elif status_string == 'S1=OFF':
            return IsegShqDevice.OUTPUT_STATUS_OFF
        elif status_string == 'S1=MAN':
            return IsegShqDevice.OUTPUT_STATUS_MANUAL
        elif status_string == 'S1=ERR':
            return IsegShqDevice.OUTPUT_STATUS_COMPLIANCE
        elif status_string == 'S1=INH':
            return IsegShqDevice.OUTPUT_STATUS_INHIBITED
        elif status_string == 'S1=QUA':
            return IsegShqDevice.OUTPUT_STATUS_BAD_QUALITY
        elif status_string == 'S1=L2H':
            return IsegShqDevice.OUTPUT_STATUS_RISING
        elif status_string == 'S1=H2L':
            return IsegShqDevice.OUTPUT_STATUS_FALLING
        elif status_string == 'S1=TRP':
            return IsegShqDevice.OUTPUT_STATUS_TRIPPED
        else:
            raise ValueError('Parser error for status_string "{0}"'.format(status_string))

    def get_device_status(self):
        """
        Read the device status word.
        A dictionary of the following elements is returned:
        remote_controlled -- True, if the control switch is set to remote control
        polarity -- +1 or -1, depending of the control dial setting for the output polarity
        output_enabled -- True, if the output switch is on
        kill_enabled -- True, if the kill switch is on
        inhibited -- True, if the inhibit signal is/was active

        """
        status_string = self.command('T1')
        status = int(status_string)

        return { 'remote_controlled' : not self._test_bit(status, 1),
                 'polarity' : +1 if self._test_bit(status, 2) else -1,
                 'output_enabled' : not self._test_bit(status, 3),
                 'kill_enabled' : self._test_bit(status, 4),
                 'inhibited' : self._test_bit(status, 5) }


# Simple connectivity test
if __name__ == "__main__":
    #device = IsegShqDevice(serial_device = '/dev/ttyS0')
    device = IsegShqDevice(usb_vendor_id = 0x0403, usb_product_id = 0x6001)
    print(device.get_device_id())
    print(device.get_output_status())
    print(device.get_device_status())
