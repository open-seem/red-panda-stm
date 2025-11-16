import serial

import numpy as np
import threading
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
        approach_direction (int): The approach direction: 1 for forward, -1 for backward.
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
    approach_direction: int = 1
    approach_recovery: bool = False
    time_millis: int = 0
    bias_scaling_factor = 1.0
    x_scaling_factor = 1.0
    y_scaling_factor = 1.0
    z_scaling_factor = 1.0

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
                          approach_direction=values[9],
                          approach_recovery=bool(values[10]),
                          time_millis=values[11])

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
    def amp_to_adc(current: float):
        """Convert current in Amperes to ADC value.

        Args:
            current (float): The current in Amperes to convert.

        Returns:
            float: The corresponding ADC value.
        """
        return current * 32768 * 100e6 / 10.24
    
    @staticmethod
    def steps_to_distance(steps: int):
        """Convert stepper motor steps to distance in micrometers.
        
        Motor: 28BYJ-48 with 4096 steps/revolution
        Lead screw: M5x0.25 (0.25mm pitch)
        
        Args:
            steps (int): Number of stepper motor steps.
            
        Returns:
            float: Distance in micrometers (μm).
        """
        STEPS_PER_REVOLUTION = 4096  # 28BYJ-48: 64 steps * 64 gear ratio
        PITCH_MM = 0.25  # M5x0.25 lead screw pitch in mm
        
        # Distance in mm
        distance_mm = (steps / STEPS_PER_REVOLUTION) * PITCH_MM
        # Convert to micrometers
        distance_um = distance_mm * 1000
        return distance_um
    
    @staticmethod
    def dac_to_piezo_displacement(dac_value: int, axis: str = 'z'):
        """Convert DAC value to piezo displacement in nanometers.
        
        Piezo: 15mm buzzer disc, 4-quadrant
        Typical displacement coefficient: 1500 nm/V
        
        DAC Configuration:
        - X axis: -5V to +5V (DAC -32768 to +32767, bipolar)
        - Y axis: -5V to +5V (DAC -32768 to +32767, bipolar)
        - Z axis: -3V to +3V (DAC -32768 to +32767, bipolar)
        
        Args:
            dac_value (int): DAC value (-32768 to 32767 for all axes)
            axis (str): 'x', 'y', or 'z'
            
        Returns:
            float: Displacement in nanometers (nm).
        """
        PIEZO_COEFFICIENT_NM_PER_V = 160.0  # 160 nm/V typical for 15mm piezo disc
        
        axis = axis.lower()
        
        if axis == 'z':
            # Z axis: -3V to +3V, bipolar, Two's complement
            # DAC range: -32768 to +32767
            voltage = (dac_value / 32768.0) * 3.0  # -3V to +3V
        elif axis == 'x':
            # X axis: -5V to +5V, bipolar, Two's complement (symmetric scanning)
            # DAC range: -32768 to +32767
            voltage = (dac_value / 32768.0) * 5.0  # -5V to +5V
        elif axis == 'y':
            # Y axis: -5V to +5V, bipolar, Two's complement (symmetric scanning)
            # DAC range: -32768 to +32767
            voltage = (dac_value / 32768.0) * 5.0  # -5V to +5V
        else:
            raise ValueError(f"Invalid axis: {axis}. Must be 'x', 'y', or 'z'")
        
        # Calculate displacement directly in nanometers
        displacement_nm = voltage * PIEZO_COEFFICIENT_NM_PER_V
        
        return displacement_nm
    
    @staticmethod
    def piezo_displacement_to_dac(displacement_nm: float, axis: str = 'z'):
        """Convert piezo displacement in nanometers to DAC value.
        
        Args:
            displacement_nm (float): Desired displacement in nanometers
            axis (str): 'x', 'y', or 'z'
            
        Returns:
            int: DAC value
        """
        PIEZO_COEFFICIENT_NM_PER_V = 1500.0
        
        axis = axis.lower()
        voltage = displacement_nm / PIEZO_COEFFICIENT_NM_PER_V
        
        if axis == 'z':
            # Z axis: -3V to +3V
            if abs(voltage) > 3.0:
                voltage = 3.0 if voltage > 0 else -3.0
            dac_value = int((voltage / 3.0) * 32768)
        elif axis == 'x':
            # X axis: -5V to +5V (bipolar, symmetric)
            if abs(voltage) > 5.0:
                voltage = 5.0 if voltage > 0 else -5.0
            dac_value = int((voltage / 5.0) * 32768)
        elif axis == 'y':
            # Y axis: -5V to +5V (bipolar, symmetric)
            if abs(voltage) > 5.0:
                voltage = 5.0 if voltage > 0 else -5.0
            dac_value = int((voltage / 5.0) * 32768)
        else:
            raise ValueError(f"Invalid axis: {axis}")
        
        return dac_value
    @staticmethod
    def set_bias_scaling_factor(factor: float):
        STM_Status.bias_scaling_factor = factor

    @staticmethod
    def dac_to_bias_volts(dac: int):
        """Convert a DAC value to the corresponding bias voltage using the scaling factor.

        Args:
            dac (int): The DAC value to convert (-32768 to 32767).

        Returns:
            float: The corresponding bias voltage.
        """
        # Base range is -10V to +10V (bipolar), multiplied by the scaling factor
        base_voltage = 10.0
        return (base_voltage * STM_Status.bias_scaling_factor) * dac / 32768

    @staticmethod
    def set_x_scaling_factor(factor: float):
        STM_Status.x_scaling_factor = factor

    @staticmethod
    def dac_to_dacx_volts(dac: int):
        """Convert a DAC value to the corresponding X-axis voltage using the scaling factor.

        Args:
            dac (int): The DAC value to convert (-32768 to 32767).

        Returns:
            float: The corresponding X-axis voltage.
        """
        # Base range is -5V to +5V (bipolar), multiplied by the scaling factor
        base_voltage = 5.0
        return (base_voltage * STM_Status.x_scaling_factor) * dac / 32768

    @staticmethod
    def set_y_scaling_factor(factor: float):
        STM_Status.y_scaling_factor = factor

    @staticmethod
    def dac_to_dacy_volts(dac: int):
        """Convert a DAC value to the corresponding Y-axis voltage using the scaling factor.

        Args:
            dac (int): The DAC value to convert (-32768 to 32767).

        Returns:
            float: The corresponding Y-axis voltage.
        """
        # Base range is -5V to +5V (bipolar), multiplied by the scaling factor
        base_voltage = 5.0
        return (base_voltage * STM_Status.y_scaling_factor) * dac / 32768

    @staticmethod
    def set_z_scaling_factor(factor: float):
        STM_Status.z_scaling_factor = factor

    @staticmethod
    def dac_to_dacz_volts(dac: int):
        """Convert a DAC value to the corresponding Z-axis voltage using the scaling factor.

        Args:
            dac (int): The DAC value to convert (-32768 to 32767).

        Returns:
            float: The corresponding Z-axis voltage.
        """
        # Base range is -3V to +3V (bipolar), multiplied by the scaling factor
        base_voltage = 3.0
        return (base_voltage * STM_Status.z_scaling_factor) * dac / 32768
    
    def to_string(self):
        """Return a formatted string representation of the STM status.

        Returns:
            str: A formatted string with all status fields.
        """
        direction_str = "Forward" if self.approach_direction == 1 else "Backward"
        approach_status = ""
        if self.is_approaching:
            if self.approach_recovery:
                approach_status = f"RECOVERING ({direction_str})"
            else:
                approach_status = f"ACTIVE ({direction_str})"
        else:
            approach_status = "IDLE"
            
        return """STM Status:
                Bias: {} 
                Z: {} 
                X: {} 
                Y: {} 
                ADC: {} 
                STEPS: {}
                Approaching: {}
                ConstCurrent: {} 
                Scan: {}  
                Time: {}""".format(self.bias, self.dac_z, self.dac_x, self.dac_y, self.adc, self.steps, 
                                 approach_status, self.is_const_current, self.is_scanning, self.time_millis)


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
        self.hist_length = 200  # Reduced history length for better performance
        self.history = deque()
        self.scan_adc = None
        self.scan_dacz = None
        self.last_steps = 0  # Track last step count for change detection

        self.scan_config = [0, 100, 10, 0, 100, 10]
        self.scan_adc = np.ones([512, 512], dtype=np.float32)
        self.scan_dacz = np.ones([512, 512], dtype=np.float32)
        # Threaded status polling
        self._status_lock = threading.Lock()
        self._poll_thread = None
        self._poll_stop = threading.Event()
        self.STATUS_POLL_INTERVAL = 0.1  # seconds

    def open(self, device):
        """Opens the serial connection to the STM device.

        Args:
            device (str): The serial port device name (e.g., 'COM3' or '/dev/ttyUSB0').
        """
        # Match firmware baudrate (Teensy uses 230400 in current firmware)
        self.stm_serial = serial.Serial(device, 230400, timeout=1)
        # set_buffer_size may not be available on all pyserial implementations; guard it
        try:
            if hasattr(self.stm_serial, 'set_buffer_size'):
                self.stm_serial.set_buffer_size(rx_size=128000, tx_size=128000)
        except Exception:
            # Non-fatal: continue without changing buffer sizes
            pass
        self.is_opened = True
        # start background status poller
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._status_polling_loop, daemon=True)
        self._poll_thread.start()

    def get_status(self):
        """Requests and retrieves the current status from the STM.

        Returns:
            STM_Status: An object containing the latest status from the STM.
                        Returns the last known status if the device is busy or
                        if there is no response.
        """
        # Return cached status quickly. The background poller keeps it updated.
        with self._status_lock:
            return self.status

    def _status_polling_loop(self):
        """Background loop: periodically request status when not busy.

        Polling is suspended while self.busy is True (e.g., during scans) to avoid
        interfering with scan data reads which use raw serial reads.
        """
        while not self._poll_stop.is_set():
            if not self.is_opened:
                time.sleep(self.STATUS_POLL_INTERVAL)
                continue

            # Do not poll while busy (scanning) to avoid stealing serial data
            if self.busy:
                time.sleep(self.STATUS_POLL_INTERVAL)
                continue

            try:
                # Request status
                self.send_cmd('GSTS')
                # Read a single status line; timeout is configured on serial object
                raw = self.stm_serial.readline().decode().strip()
                if not raw:
                    time.sleep(self.STATUS_POLL_INTERVAL)
                    continue
                parts = raw.split(',')
                if len(parts) >= 11:
                    vals = [int(x) for x in parts]
                    s = STM_Status.from_list(vals)
                    with self._status_lock:
                        self.status = s
                        # maintain history
                        if not self.history or self.status.steps != self.last_steps or abs(self.status.adc - self.history[-1].adc) > 5:
                            self.history.append(self.status)
                            self.last_steps = self.status.steps
                            if len(self.history) > self.hist_length:
                                self.history.popleft()
            except Exception:
                # Ignore parse/read errors; keep polling
                pass

            time.sleep(self.STATUS_POLL_INTERVAL)

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
        self.send_cmd(f'MTMV {steps}')
        # Immediately update status after motor move for faster GUI response
        time.sleep(0.01)  # Small delay to let motor command execute
        self.get_status()

    def approach(self, target_dac, max_steps, step_interval, direction=1):
        """Initiates the tip approach procedure.

        Args:
            target_dac (int): The target DAC value for the approach.
            max_steps (int): The maximum number of motor steps to take.
            step_interval (int): The number of steps to take in each iteration.
            direction (int): The approach direction: 1 for forward, -1 for backward.
        """
        direction = 1 if direction >= 0 else -1  # Ensure only 1 or -1
        self.send_cmd(f'APRH {target_dac} {max_steps} {step_interval} {direction}')

    def stop(self):
        """Sends a command to stop any ongoing operation on the STM."""
        self.send_cmd('STOP')

    def stepper_stop(self):
        """Sends a command to stop the stepper motor."""
        self.send_cmd('STPS')


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

    def get_pid_debug(self):
        """Gets PID debug information from the STM.
        
        Returns:
            str: PID debug information string
        """
        if self.is_opened:
            self.send_cmd('PIDD')
            debug_str = self.stm_serial.readline().decode().strip()
            return debug_str
        return "Not connected"

    def get_approach_debug(self):
        """Gets approach debug information from the STM.
        
        Returns:
            str: Approach debug information string
        """
        if self.is_opened:
            self.send_cmd('APRD')
            debug_str = self.stm_serial.readline().decode().strip()
            return debug_str
        return "Not connected"

    def trigger_approach_recovery(self):
        """Manually triggers approach recovery mode for testing."""
        if self.is_opened:
            self.send_cmd('APRR')
            print("Approach recovery triggered")

    def enable_fine_motor_mode(self):
        """Enable fine motor mode on the device (single-step motor between Z sweeps)."""
        if self.is_opened:
            # Send explicit enable (1)
            self.send_cmd('FINE 1')
            print("Fine motor mode command sent")

    def disable_fine_motor_mode(self):
        """Disable fine motor mode (coarse mode)."""
        if self.is_opened:
            self.send_cmd('FINE 0')
            print("Fine motor mode disabled")

    def set_approach_fine_step_size(self, step_size: int):
        """Set the piezo Z step size used in fine approach sweeps."""
        if self.is_opened:
            self.send_cmd(f'APFS {int(step_size)}')
            print(f"Sent approach fine step size: {step_size}")

    def set_approach_z_range(self, z_range: int):
        """Set the Z sweep search range used for approach."""
        if self.is_opened:
            self.send_cmd(f'APRG {int(z_range)}')
            print(f"Sent approach Z search range: {z_range}")

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

