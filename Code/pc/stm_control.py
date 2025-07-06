import serial

import numpy as np
from dataclasses import dataclass
from collections import deque
import time


@dataclass
class STM_Status:
    """A dataclass to hold the status of the STM.

    Attributes:
        bias (int): The current bias DAC value.
        dac_z (int): The current Z-axis DAC value.
        dac_x (int): The current X-axis DAC value.
        dac_y (int): The current Y-axis DAC value.
        adc (int): The current ADC reading.
        steps (int): The number of motor steps taken.
        is_approaching (bool): True if the STM is in approach mode.
        is_const_current (bool): True if the STM is in constant current mode.
        is_scanning (bool): True if the STM is scanning.
        time_millis (int): The timestamp of the status in milliseconds.
    """
    bias: int = 0
    dac_z: int = 0
    dac_x: int = 0
    dac_y: int = 0
    adc: int = 0
    steps: int = 0
    is_approaching: bool = False
    is_const_current: bool = False
    is_scanning: bool = False
    time_millis: int = 0

    @staticmethod
    def from_list(values):
        """Create an STM_Status object from a list of values.

        Args:
            values (list): A list of status values from the STM.

        Returns:
            STM_Status: An instance of the STM_Status class.
        """
        return STM_Status(bias=values[0],
                          dac_z=values[1],
                          dac_x=values[2],
                          dac_y=values[3],
                          adc=values[4],
                          steps=values[5],
                          is_approaching=bool(values[6]),
                          is_const_current=bool(values[7]),
                          is_scanning=bool(values[8]),
                          time_millis=values[9])

    @staticmethod
    def adc_to_amp(adc: int):
        """Convert an ADC value to current in Amperes.

        Args:
            adc (int): The ADC value to convert.

        Returns:
            float: The corresponding current in Amperes.
        """
        return 1.0 * adc / 32768 * 10.24 / 100e6

    @staticmethod
    def dac_to_dacz_volts(dac: int):
        """Convert a DAC value to the corresponding Z-axis voltage.

        Args:
            dac (int): The DAC value to convert.

        Returns:
            float: The corresponding voltage for the Z-axis.
        """
        return 1.0 * (dac - 32768) / 32768 * 10.0 / 2.0

    @staticmethod
    def dac_to_dacx_volts(dac: int):
        """Convert a DAC value to the corresponding X-axis voltage.

        Args:
            dac (int): The DAC value to convert.

        Returns:
            float: The corresponding voltage for the X-axis.
        """
        return 1.0 * (dac - 32768) / 32768 * 10.0 / 2.0

    @staticmethod
    def dac_to_dacy_volts(dac: int):
        """Convert a DAC value to the corresponding Y-axis voltage.

        Args:
            dac (int): The DAC value to convert.

        Returns:
            float: The corresponding voltage for the Y-axis.
        """
        return 1.0 * (dac - 32768) / 32768 * 10.0 / 2.0

    @staticmethod
    def dac_to_bias_volts(dac: int):
        """Convert a DAC value to the corresponding bias voltage.

        Args:
            dac (int): The DAC value to convert.

        Returns:
            float: The corresponding bias voltage.
        """
        # Unipolar 0V to +3V range
        return 3.0 * dac / 65535.0

    def to_string(self):
        """Return a formatted string representation of the STM status.

        Returns:
            str: A formatted string with all status fields.
        """
        return """STM Status:
                Bias: {} 
                Z: {} 
                X: {} 
                Y: {} 
                ADC: {} 
                STEPS: {}
                Appoaching: {} 
                ConstCurrent: {} 
                Scan: {}  
                Time: {}""".format(self.bias, self.dac_z, self.dac_x, self.dac_y, self.adc, self.steps, self.is_approaching,  self.is_const_current, self.is_scanning, self.time_millis)


class STM(object):
    """Provides a high-level interface for controlling the STM device."""
    def __init__(self, device=None):
        """Initializes the STM controller.

        Args:
            device (str, optional): The serial port device name. 
                                    If provided, opens the connection on init.
                                    Defaults to None.
        """
        self.is_opened = False
        self.busy = False
        if device:
            self.open(device)

        self.status = STM_Status()
        self.hist_length = 1000
        self.history = deque()
        self.scan_adc = None
        self.scan_dacz = None

        self.scan_config = [0, 100, 10, 0, 100, 10]
        self.scan_adc = np.ones([512, 512], dtype=np.float32)
        self.scan_dacz = np.ones([512, 512], dtype=np.float32)

    def open(self, device):
        """Opens the serial connection to the STM device.

        Args:
            device (str): The serial port device name (e.g., 'COM3' or '/dev/ttyUSB0').
        """
        self.stm_serial = serial.Serial(device, 115200, timeout=1)
        self.stm_serial.set_buffer_size(rx_size=128000, tx_size=128000)
        self.is_opened = True

    def get_status(self):
        """Requests and retrieves the current status from the STM.

        Returns:
            STM_Status: An object containing the latest status from the STM.
                        Returns the last known status if the device is busy or
                        if there is no response.
        """
        if self.busy:
            return

        if self.is_opened:
            try:
                self.send_cmd('GSTS')
                status_str = self.stm_serial.readline().decode()
                if not status_str:
                    # No response from STM, return last known status if available
                    if self.history:
                        return self.history[-1]
                    return self.status
                status_value = status_str.split(',')
                status_value = [int(x) for x in status_value]
                self.status = STM_Status.from_list(status_value)
            except (ValueError, IndexError, serial.SerialException) as e:
                print(f'Error parsing status or serial communication failed: {e}')
                if self.history:
                    return self.history[-1]
                return self.status
        else:
            # Not connected, return default status
            self.status = STM_Status()

        self.history.append(self.status)
        if len(self.history) > self.hist_length:
            self.history.popleft()

        return self.status

    def reset(self):
        """Sends a reset command to the STM and clears the status history."""
        self.send_cmd('RSET')
        self.clear()

    def clear(self):
        """Clears the internal history of STM status objects."""
        self.history = deque()

    def send_cmd(self, cmd):
        """Sends a command string to the STM via the serial port.

        Args:
            cmd (str): The command to send.
        """
        if self.is_opened:
            self.stm_serial.write(cmd.encode())

    def move_motor(self, steps):
        """Moves the stepper motor by a specified number of steps.

        Args:
            steps (int): The number of steps to move the motor.
        """
        self.send_cmd('MTMV {steps}')

    def approach(self, target_dac, steps):
        """Initiates the tip approach procedure.

        Args:
            target_dac (int): The target DAC value for the approach.
            steps (int): The number of steps for the approach motor.
        """
        self.send_cmd(f'APRH {target_dac} {steps}')

    def stop(self):
        """Sends a command to stop any ongoing operation on the STM."""
        self.send_cmd('STOP')

    def stepper_stop(self):
        """Sends a command to stop the stepper motor."""
        self.send_cmd('STPS')

    def stepper_step_forward(self):
        """Sends a command to move the stepper motor one step forward."""
        self.send_cmd('STPF')

    def stepper_step_backward(self):
        """Sends a command to move the stepper motor one step backward."""
        self.send_cmd('STPB')


    def measure_iv_curve(self, dac_start, dac_end, dac_step):
        """Initiates an I-V curve measurement and retrieves the data.

        Args:
            dac_start (int): The starting DAC value for the bias sweep.
            dac_end (int): The ending DAC value for the bias sweep.
            dac_step (int): The step size for the DAC value sweep.

        Returns:
            list: A list of integers representing the I-V curve data.
        """
        self.send_cmd(f'IVME {dac_start} {dac_end} {dac_step}')
        # Wait for 0.1s for the STM to response
        time.sleep(2)
        return self.get_iv_curve()

    def get_iv_curve(self):
        """Retrieves the last measured I-V curve data from the STM.

        Returns:
            list: A list of integers representing the I-V curve data.
        """
        iv_curve_values = [0, 0]
        if self.is_opened:
            self.busy = True
            time.sleep(1)
            self.send_cmd('IVGE')
            data_str = self.stm_serial.readline().decode()
            data = data_str.split(',')
            if data[0] == "IV":
                iv_curve_values = [int(x) for x in data[1:]]
        self.busy = False
        print(iv_curve_values)
        return iv_curve_values

    def set_bias(self, value):
        """Sets the bias DAC to a specific value.

        Args:
            value (int): The DAC value to set for the bias.
        """
        self.send_cmd(f"BIAS {value}")

    def set_dacz(self, value):
        """Sets the Z-axis DAC to a specific value.

        Args:
            value (int): The DAC value to set for the Z-axis.
        """
        self.send_cmd(f"DACZ {value}")

    def set_dacx(self, value):
        """Sets the X-axis DAC to a specific value.

        Args:
            value (int): The DAC value to set for the X-axis.
        """
        self.send_cmd(f"DACX {value}")

    def set_dacy(self, value):
        """Sets the Y-axis DAC to a specific value.

        Args:
            value (int): The DAC value to set for the Y-axis.
        """
        self.send_cmd(f"DACY {value}")

    def turn_on_const_current(self, target_adc):
        """Enables the constant current feedback mode.

        Args:
            target_adc (int): The target ADC value for the feedback loop.
        """
        self.send_cmd(f"CCON {target_adc}")

    def turn_off_const_current(self):
        """Disables the constant current feedback mode."""
        self.send_cmd("CCOF")

    def set_pid(self, Kp, Ki, Kd):
        """Sets the parameters for the PID feedback controller.

        Args:
            Kp (float): The proportional gain.
            Ki (float): The integral gain.
            Kd (float): The derivative gain.
        """
        self.send_cmd(f"PIDS {Kp} {Ki} {Kd}")

    def start_scan(self, x_start, x_end, x_resolution, y_start, y_end, y_resolution, sample_number):
        """Starts a 2D scan and collects data.

        Args:
            x_start (int): The starting DAC value for the X-axis.
            x_end (int): The ending DAC value for the X-axis.
            x_resolution (int): The number of points to scan in the X-axis.
            y_start (int): The starting DAC value for the Y-axis.
            y_end (int): The ending DAC value for the Y-axis.
            y_resolution (int): The number of points to scan in the Y-axis.
            sample_number (int): The number of samples to average at each point.
        """
        self.busy = True
        self.scan_config = [x_start, x_end,
                            x_resolution, y_start, y_end, y_resolution]
        self.send_cmd(
            f"SCST {x_start} {x_end} {x_resolution} {y_start} {y_end} {y_resolution} {sample_number}")

        self.scan_adc = np.ones([x_resolution, y_resolution], dtype=np.float32)
        self.scan_dacz = np.ones(
            [x_resolution, y_resolution], dtype=np.float32)

        current_line = ''

        def _process_full_line(full_line):
            # print(full_line)
            data = full_line.split(',')
            data_type = data[0]
            if data_type == "A":
                x_i = int(data[1])
                data_content = data[2:]
                data_content = [int(x) for x in data_content]
                self.scan_adc[x_i, :] = data_content
            if data_type == "Z":
                x_i = int(data[1])
                data_content = data[2:]
                data_content = [int(x) for x in data_content]
                self.scan_dacz[x_i, :] = data_content
            if data_type == "D":
                return True
            return False

        while (True):
            read_number = self.stm_serial.inWaiting()
            if (read_number == 0):
                continue
            read_str = self.stm_serial.read(read_number).decode()
            if "\n" in read_str:
                split_lines = read_str.split("\n")
                for data_line in split_lines:
                    if len(data_line) == 0:
                        continue
                    current_line += data_line
                    if current_line[-1] == "\r":  # We have a full line
                        _process_full_line(current_line)
                        current_line = ''
            else:
                current_line += read_str
            # We have a full line
            if current_line and current_line[-1] == "\r":
                _process_full_line(data_line)
                current_line = ''
            if "D" in read_str:
                break
        self.busy = False
        return
