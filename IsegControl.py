#!/usr/bin/env python

#
# IsegControl.py - ISEG SHQ command line controller
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

import IsegShqDevice
import cmd
import time
import datetime
import os


class IsegControlInterpreter(cmd.Cmd):
    prompt = 'ISEG > '

    def do_apply_preset(self, argument):
        """
        Apply the voltage_preset to the output of the SHQ device.

        """
        arguments = argument.split()
        if len(arguments) != 0:
            print('Usage: apply_preset')
        else:
            device.apply_voltage_preset()

    def do_set(self, argument):
        """
        Set a parameter on the SHQ device.

        Parameters:
        communication_delay -- Delay between each character sent from the device to the computer (in ms)
        voltage_preset -- Voltage preset to be applied to the device output, without polarity sign (in V)
        voltage_ramping_speed -- Voltage ramping speed (2..255 V/s) when increasing or decreasing the output voltage (in V/s)

        """
        arguments = argument.split()
        if len(arguments) != 2:
            print('Usage: set parameter value')
        else:
            parameter_name = arguments[0]
            parameter_value = arguments[1]

            if parameter_name == 'communication_delay':
                device.set_communication_delay(int(parameter_value))
            elif parameter_name == 'voltage_preset':
                voltage_preset = float(parameter_value)
                if voltage_preset < 0:
                    raise ValueError
                else:
                    device.set_voltage_preset(voltage_preset)
            elif parameter_name == 'voltage_ramping_speed':
                device.set_voltage_ramping_speed(int(parameter_value))

            self.prompt = 'ISEG (U={0} V, I={1} uA) > '.format(device.get_voltage(), device.get_current()*1e6)

    def complete_set(self, text, line, begidx, endidx):
        return [parameter_name for parameter_name in ('communication_delay', 'voltage_preset', 'voltage_ramping_speed') if parameter_name.startswith(text)]

    def do_get(self, argument):
        """
        Get a parameter from the SHQ device.

        Parameters:
        communication_delay -- Delay between each character sent from the device to the computer (in ms)
        voltage -- Voltage present on the output (in V)
        current -- Current flowing through the output (in A)
        voltage_limit -- Hardware voltage limit as set with the Vmax dial on the device (in V)
        current_limit -- Hardware current limit as set with the Iset dial on the device (in A)
        voltage_preset -- Voltage preset to be applied to the device output (in V)
        voltage_ramping_speed -- Voltage ramping speed (2..255 V/s) when increasing or decreasing the output voltage (in V/s)

        """
        arguments = argument.split()
        if len(arguments) != 1:
            print('Usage: get parameter')
        else:
            parameter_name = arguments[0]

            if parameter_name == 'communication_delay':
                print(device.get_communication_delay())
            elif parameter_name == 'voltage':
                print('{0} V'.format(device.get_voltage()))
            elif parameter_name == 'current':
                print('{0} uA'.format(device.get_current() * 1e6))
            elif parameter_name == 'voltage_limit':
                print('{0} V'.format(device.get_voltage_limit() * device.get_device_id()['maximum_voltage']))
            elif parameter_name == 'current_limit':
                print('{0} uA'.format(device.get_current_limit() * device.get_device_id()['maximum_current'] * 1e6))
            elif parameter_name == 'voltage_preset':
                print('{0} V'.format(device.get_voltage_preset()))
            elif parameter_name == 'voltage_ramping_speed':
                print('{0} V/s'.format(device.get_voltage_ramping_speed()))

            self.prompt = 'ISEG (U={0} V, I={1} uA) > '.format(device.get_voltage(), device.get_current() * 1e6)

    def complete_get(self, text, line, begidx, endidx):
        return [parameter_name for parameter_name in ('communication_delay', 'voltage', 'current', 'voltage_limit', 'current_limit', 'voltage_preset', 'voltage_ramping_speed') if parameter_name.startswith(text)]

    def do_scan(self, argument):
        """
        Scan the voltage and record voltage/current measurements.

        Parameters
        start_voltage -- Initial voltage to set and start scanning from (in V)
        stop_voltage -- Final voltage to scan to (in V)
        step_count -- Number of steps
        measurement_count -- Number of measurements to do in each step
        settling_time -- Seconds to wait between applying the voltage preset and taking the measurement

        """
        arguments = argument.split()
        if len(arguments) != 4 and len(arguments) != 5:
            print('Usage: scan start_voltage stop_voltage step_count measurement_count [settling_time]')
        else:
            start_voltage = float(arguments[0])
            if start_voltage < 0:
                raise ValueError
            stop_voltage = float(arguments[1])
            if stop_voltage < 0:
                raise ValueError
            step_count = int(arguments[2])
            if step_count < 1:
                raise ValueError
            measurement_count = int(arguments[3])
            if measurement_count < 1:
                raise ValueError
            
            if len(arguments) == 5:
                settling_time = int(arguments[4])
            else:
                settling_time = 0

            scan_range = stop_voltage - start_voltage
            step_size = scan_range / step_count

            if settling_time > 0:
                print('Scanning the voltage from {0:.2f} V to {1:.2f} V in {2} steps of {3:.2f} V. Wait {4} s for the system to settle, then take {5} measurement(s) per step'.format(start_voltage, stop_voltage, step_count, step_size, settling_time, measurement_count))
            else:
                print('Scanning the voltage from {0:.2f} V to {1:.2f} V in {2} steps of {3:.2f} V. Take {4} measurement(s) per step'.format(start_voltage, stop_voltage, step_count, step_size, measurement_count))


            log_file_name = 'iseghv_scan_{0}.dat'.format(datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
            log_file_full_name = os.path.join(os.getcwd(), log_file_name)

            print('Logging scan to file "{0}"'.format(log_file_name))

            with open(log_file_full_name, 'w') as logfile:
                logfile.write('# ISEG HV scan log file\n')
                logfile.write('# start_voltage={0} V, stop_voltage={1} V, step_count={2}, step_size={3} V, settling_time={4} s, measurement_count={5}\n'.format(start_voltage, stop_voltage, step_count, step_size, settling_time, measurement_count))
                logfile.write('# Upreset (V)  Umeasured (V)  Imeasured (A)\n')

                for step_number in range(0, step_count+1):
                    step_voltage = start_voltage + step_number * step_size
                    device.set_voltage_preset(step_voltage)
                    device.apply_voltage_preset()
                    while device.get_output_status() != device.OUTPUT_STATUS_ON:
                        time.sleep(1)
                    time.sleep(settling_time)
                    for measurement_number in range(0, measurement_count):
                        measured_voltage_preset = device.get_voltage_preset()
                        measured_voltage = device.get_voltage()
                        measured_current = device.get_current()
                        print('Upreset={0} V, U={1} V, I={2} uA'.format(measured_voltage_preset, measured_voltage_preset, measured_current * 1e6))
                        logfile.write('{0} {1} {2}\n'.format(measured_voltage_preset, measured_voltage, measured_current))




    def do_EOF(self, argument):
        print('\n\nEOF received, terminating.')
        return True

    def emptyline(self):
        self.prompt = 'ISEG (U={0} V, I={1} uA) > '.format(device.get_voltage(), device.get_current()*1e6)

def main():
    # USB VID/PID for single channel FTDI USB<->RS232 converter
    usb_vendor_id = 0x0403
    usb_product_id = 0x6001

    global device
    device = IsegShqDevice.IsegShqDevice(usb_vendor_id = usb_vendor_id, usb_product_id = usb_product_id)
    #device = IsegShqDevice.IsegShqDevice(serial_device='/dev/ttyS0')

    # Default settings
    device.set_communication_delay(1)
    device.set_voltage_ramping_speed(100)
    device_id = device.get_device_id()
    device_status = device.get_device_status()

    print('Welcome to the ISEG SHQ command line controller!')
    print('------------------------------------------------')
    print()
    print('Device information:')
    print(' - Identification number: {0}'.format(device_id['identification_number']))
    print(' - Software version: {0}'.format(device_id['software_version']))
    print(' - Maximum voltage: {0} V'.format(device_id['maximum_voltage']))
    print(' - Maximum current: {0} uA'.format(device_id['maximum_current'] * 1e6))
    print(' - Output polarity: {0}'.format('Positive' if device_status['polarity'] == +1 else 'Negative'))
    print(' - Output enabled: {0}'.format(device_status['output_enabled']))
    print(' - Kill enabled: {0}'.format(device_status['kill_enabled']))
    print(' - Inhibited: {0}'.format(device_status['inhibited']))
    print()

    if device_status['remote_controlled'] == True:
        IsegControlInterpreter().cmdloop()
    else:
        print('Device is not set to be remote controlled. Exiting')

if __name__ == '__main__':
    device = None # Use as global object in main method but also in the interpreter class
    main()
