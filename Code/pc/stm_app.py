from datetime import datetime
import tkinter as tk
from tkinter import ttk
import numpy as np
import stm_control
import time
import csv
import threading
import serial.tools.list_ports


from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure


def save_data_to_file(filename_prefix, data_to_store: list):
    current_time_stamp = datetime.now()
    ts = int(datetime.timestamp(current_time_stamp)*1000)
    with open(f"{filename_prefix}_{ts}.csv", 'w', newline='') as csvfile:
        datawriter = csv.writer(csvfile)
        datawriter.writerow(['Voltage (V)', 'Current (A)'])
        for data in data_to_store:
            datawriter.writerow(data)


class PlotFrame(ttk.Frame):
    def __init__(self, parent, with_toobar=False,  dpi=100.0, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.figure = Figure(dpi=dpi, layout='tight')

        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side='top', fill='both', expand=True)

        if with_toobar:
            self.toolbar = NavigationToolbar2Tk(self.canvas, self, pack_toolbar=False)
            self.toolbar.update()
            self.toolbar.pack(side=tk.BOTTOM, fill=tk.X)

    def add_plot(self, label=None, xlabel=None, ylabel=None,):
        self.ax = self.figure.add_subplot(111)
        self.plot = self.ax.plot([0], [0], '-', label=label)[0]
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        if xlabel:
            self.ax.set(xlabel=xlabel)
        if ylabel:
            self.ax.set(ylabel=ylabel)

    def add_image(self, image, title=None):
        self.ax = self.figure.add_subplot(111)
        if title:
            self.ax.set_title(title)
        
        self.image = self.ax.imshow(image, interpolation='nearest', norm='linear', origin="lower", aspect='equal', cmap='jet')
        self.cbar = self.figure.colorbar(self.image, ax=self.ax, fraction=0.046, pad=0.04)


    def update_plot(self, x_data, y_data):
        if not hasattr(self, 'plot'): return
        self.plot.set_xdata(x_data)
        self.plot.set_ydata(y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()
        self.canvas.flush_events()

    def update_image(self, image_data, extent=None):
        if not hasattr(self, 'image'): return
        self.image.set_data(image_data)
        if image_data.size > 0:
            min_val, max_val = np.min(image_data), np.max(image_data)
            if min_val < max_val:
                self.image.set_clim(min_val, max_val)
        
        self.image.axes.autoscale_view()
        self.canvas.draw()
        self.canvas.flush_events()

    def save_figure(self, image_path):
        self.figure.savefig(image_path)


class MotorControl(ttk.Frame):
    def __init__(self, parent, stm_control, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.stm = stm_control
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_columnconfigure(3, weight=1)

        self.steps_var = tk.StringVar(value="10")

        back_button = ttk.Button(self, text="Back", command=self._move_backward)
        back_button.grid(row=0, column=0, sticky='ew', padx=1)

        steps_entry = ttk.Entry(self, textvariable=self.steps_var, width=8)
        steps_entry.grid(row=0, column=1, sticky='ew', padx=1)

        forward_button = ttk.Button(self, text="Forward", command=self._move_forward)
        forward_button.grid(row=0, column=2, sticky='ew', padx=1)

        stop_button = ttk.Button(self, text="Stop", command=self.stm.stepper_stop)
        stop_button.grid(row=0, column=3, sticky='ew', padx=1)

    def _move_backward(self):
        try:
            steps = int(self.steps_var.get())
            self.stm.move_motor(-steps)
        except ValueError:
            self.steps_var.set("10")

    def _move_forward(self):
        try:
            steps = int(self.steps_var.get())
            self.stm.move_motor(steps)
        except ValueError:
            self.steps_var.set("10")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.stm = stm_control.STM()
        self.wm_title("Panda STM")
        self.is_const_current_on = False
        
        self.style = ttk.Style(self)
        self.style.map('Blue.TButton',
            background=[('active', '#00599c'), ('!disabled', '#0078d4')],
            foreground=[('active', 'white'), ('!disabled', 'white')]
        )

        main_pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True)

        control_outer_frame = ttk.Frame(main_pane)
        main_pane.add(control_outer_frame, weight=0)
        control_outer_frame.grid_rowconfigure(0, weight=1)
        control_outer_frame.grid_columnconfigure(0, weight=1)
        canvas = tk.Canvas(control_outer_frame)
        canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar = ttk.Scrollbar(control_outer_frame, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky='ns')
        canvas.configure(yscrollcommand=scrollbar.set)
        self.control_frames = ttk.Frame(canvas, padding=10)
        canvas_frame_id = canvas.create_window((0, 0), window=self.control_frames, anchor="nw")
        def on_frame_configure(event): canvas.configure(scrollregion=canvas.bbox("all"))
        def on_canvas_configure(event): canvas.itemconfig(canvas_frame_id, width=event.width)
        self.control_frames.bind("<Configure>", on_frame_configure)
        canvas.bind("<Configure>", on_canvas_configure)

        notebook = ttk.Notebook(main_pane)
        main_pane.add(notebook, weight=1)

        curve_tab_frame = ttk.Frame(notebook, padding=5)
        image_tab_frame = ttk.Frame(notebook, padding=5)
        notebook.add(curve_tab_frame, text="Curves")
        notebook.add(image_tab_frame, text="Image")

        curve_tab_frame.grid_columnconfigure(0, weight=1)
        curve_tab_frame.grid_columnconfigure(1, weight=2)
        curve_tab_frame.grid_rowconfigure(0, weight=1)
        curve_tab_frame.grid_rowconfigure(1, weight=1)

        # --- Configure Grid for Image Tab ---
        image_tab_frame.grid_columnconfigure(0, weight=1) # First column
        image_tab_frame.grid_columnconfigure(1, weight=1) # Second column
        image_tab_frame.grid_rowconfigure(0, weight=1)    # Single row

        # --- Place plots into the "Curves" tab ---
        self.real_time_current_plot_frame = PlotFrame(curve_tab_frame, with_toobar=True)
        self.real_time_current_plot_frame.add_plot(label="Current", xlabel='Time (s)', ylabel='Current (A)')
        self.real_time_current_plot_frame.grid(row=0, column=0, padx=5, pady=5, sticky='nsew')

        self.real_time_steps_plot_frame = PlotFrame(curve_tab_frame, with_toobar=True)
        self.real_time_steps_plot_frame.add_plot(label="Z Steps", xlabel='Time (s)', ylabel='Piezo Steps')
        self.real_time_steps_plot_frame.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        self.iv_curve_frame = PlotFrame(curve_tab_frame, with_toobar=True)
        self.iv_curve_frame.add_plot(label="IV Curve", xlabel="Bias (V)", ylabel="Current (A)")
        self.iv_curve_frame.grid(row=0, column=1, padx=5, pady=5, sticky='nsew')

        # --- Place plots into the "Image" tab ---
        self.scan_dacz_frame = PlotFrame(image_tab_frame, with_toobar=True)
        self.scan_dacz_frame.add_image(np.zeros((10, 10)), title="DAC Z Output")
        self.scan_dacz_frame.grid(row=0, column=0, padx=5, pady=5, sticky='nsew') # row 0, col 0
        
        self.scan_adc_frame = PlotFrame(image_tab_frame, with_toobar=True)
        self.scan_adc_frame.add_image(np.zeros((10, 10)), title="ADC Reading (Topography)")
        self.scan_adc_frame.grid(row=0, column=1, padx=5, pady=5, sticky='nsew') # row 0, col 1


        self.setup_control_widgets()
        self._update_images()
        self._update_real_time()
        self.state('zoomed')
        self.after_idle(lambda: main_pane.sashpos(0, 400))

    def setup_control_widgets(self):
        self.control_frames.grid_columnconfigure(0, weight=1)
        row_number = 0

        class _DAC_Slider_Control(ttk.Frame):
            def __init__(self, parent, text, default_value, cmd_func, convert_func, from_, to, *args, **kwargs):
                super().__init__(parent, *args, **kwargs)
                self.grid_columnconfigure(1, weight=1)
                self.cmd_func, self.convert_func = cmd_func, convert_func
                self.variable = tk.DoubleVar(value=default_value)

                self.label = ttk.Label(self, text=text, width=5)
                self.label.grid(row=0, column=0, sticky=tk.W)

                self.slider = ttk.Scale(self, from_=from_, to=to, orient=tk.HORIZONTAL, variable=self.variable, command=self._on_slider_change)
                self.slider.grid(row=0, column=1, sticky='ew', padx=5)

                self.input_string_var = tk.StringVar(value=str(default_value))
                self.input_entry = ttk.Entry(self, textvariable=self.input_string_var, width=7)
                self.input_entry.grid(row=0, column=2, padx=5)
                self.input_entry.bind('<Return>', self._on_entry_set)
                self.input_entry.bind('<FocusOut>', self._on_entry_set)

                self.display_var = tk.StringVar()
                self.display = ttk.Label(self, textvariable=self.display_var, width=10, anchor='w')
                self.display.grid(row=0, column=3, padx=5)
                
                self._update_display(default_value)

            def _on_slider_change(self, value_str):
                value = int(float(value_str))
                self.input_string_var.set(str(value))
                self._update_display(value)
                self.cmd_func(value)

            def _on_entry_set(self, event=None):
                try:
                    value = int(self.input_string_var.get())
                    from_ = self.slider.cget('from')
                    to = self.slider.cget('to')
                    if value < from_: value = int(from_)
                    if value > to: value = int(to)
                    self.variable.set(value)
                    self.input_string_var.set(str(value))
                    self._update_display(value)
                    self.cmd_func(value)
                except (ValueError, TypeError):
                    current_val = int(self.variable.get())
                    self.input_string_var.set(str(current_val))

            def _update_display(self, value):
                try:
                    voltage = self.convert_func(int(value))
                    self.display_var.set(f"{voltage:.4f} V")
                except (ValueError, TypeError):
                    self.display_var.set("Invalid")

        class _ButtonWithEntry(ttk.Frame):
            def __init__(self, parent, text, default_value_list, cmd_func, display_list=None, *args, **kwargs):
                super().__init__(parent, *args, **kwargs)
                self.grid_columnconfigure(1, weight=1)
                self.cmd_func, self.input_string_var_list = cmd_func, []
                button = ttk.Button(master=self, text=text, command=self._set_values)
                button.grid(row=0, column=0, sticky=tk.W, rowspan=2 if display_list else 1, padx=(0, 5))
                controls_frame = ttk.Frame(self)
                controls_frame.grid(row=0, column=1, rowspan=2 if display_list else 1, sticky='ew')
                for i, dv in enumerate(default_value_list):
                    controls_frame.grid_columnconfigure(i, weight=1)
                    if display_list and i < len(display_list):
                        ttk.Label(master=controls_frame, text=display_list[i]).grid(row=0, column=i, sticky=tk.W)
                    var = tk.StringVar(value=dv)
                    ttk.Entry(controls_frame, textvariable=var).grid(row=1 if display_list else 0, column=i, sticky='ew', padx=1)
                    self.input_string_var_list.append(var)
            def _set_values(self): self.cmd_func(*[var.get() for var in self.input_string_var_list])
        
        class _MultipleButtons(ttk.Frame):
            def __init__(self, parent, text_list, func_list, *args, **kwargs):
                super().__init__(parent, *args, **kwargs)
                for i in range(len(text_list)): self.grid_columnconfigure(i, weight=1)
                for i, (text, func) in enumerate(zip(text_list, func_list)):
                    ttk.Button(master=self, text=text, command=func).grid(row=0, column=i, sticky='ew', padx=1)
        
        class _ScanControl(ttk.Frame):
            def __init__(self, parent, cmd_func, *args, **kwargs):
                super().__init__(parent, *args, **kwargs)
                self.grid_columnconfigure(1, weight=1)
                self.var_list = []
                self.scan_button = ttk.Button(self, text="Scan", command=lambda: cmd_func(*[int(v.get()) for v in self.var_list]))
                self.scan_button.grid(row=0, column=0, rowspan=4, padx=(0,5), sticky='ns')
                params_frame = ttk.Frame(self)
                params_frame.grid(row=0, column=1, sticky='ew')
                for i in range(1, 4): params_frame.grid_columnconfigure(i, weight=1)
                labels = ["Start", "End", "Interval"]; defaults = [["31768", "33768", "512"], ["31768", "33768", "512"]]
                ttk.Label(params_frame, text="X:").grid(row=1, column=0, sticky='e'); ttk.Label(params_frame, text="Y:").grid(row=2, column=0, sticky='e')
                for i, label in enumerate(labels):
                    ttk.Label(params_frame, text=label).grid(row=0, column=i+1, sticky='w')
                for r, row_defs in enumerate(defaults):
                    for c, val in enumerate(row_defs):
                        var = tk.StringVar(value=val)
                        ttk.Entry(params_frame, textvariable=var).grid(row=r+1, column=c+1, sticky='ew', padx=1)
                        self.var_list.append(var)
                ttk.Label(params_frame, text="Samples:").grid(row=3, column=1, sticky='e', pady=(5,0))
                sample_var = tk.StringVar(value="10")
                ttk.Entry(params_frame, textvariable=sample_var).grid(row=3, column=2, pady=(5,0), sticky='ew')
                self.var_list.append(sample_var)
        
        def add_control_widget(widget_class, *args, **kwargs):
            nonlocal row_number
            widget_frame = widget_class(self.control_frames, *args, **kwargs)
            widget_frame.grid(row=row_number, column=0, pady=2, sticky='ew')
            row_number += 1
            return widget_frame
        
        def add_separator():
            nonlocal row_number
            ttk.Separator(self.control_frames, orient='horizontal').grid(row=row_number, column=0, sticky='ew', pady=5)
            row_number += 1

        # --- Connection Control ---
        connection_frame = ttk.Frame(self.control_frames)
        connection_frame.grid(row=row_number, column=0, pady=2, sticky='ew')
        connection_frame.grid_columnconfigure(1, weight=1) # Let the combobox expand
        row_number += 1

        open_button = ttk.Button(connection_frame, text="Open", command=lambda: self.stm.open(self.port_var.get()))
        open_button.grid(row=0, column=0, sticky='w', padx=(0,5))

        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_var = tk.StringVar(value=ports[0] if ports else "")
        port_combobox = ttk.Combobox(connection_frame, textvariable=self.port_var, values=ports, state="readonly")
        port_combobox.grid(row=0, column=1, sticky='ew')

        def refresh_ports():
            new_ports = [port.device for port in serial.tools.list_ports.comports()]
            port_combobox['values'] = new_ports
            if self.port_var.get() not in new_ports:
                self.port_var.set(new_ports[0] if new_ports else "")

        refresh_button = ttk.Button(connection_frame, text="↻", command=refresh_ports, width=3)
        refresh_button.grid(row=0, column=2, sticky='e', padx=(5,0))
        # --- NEW: Added "Help" button ---
        add_control_widget(_MultipleButtons, ["STOP", "Reset", "Clear", "Help"],
                           [self.stm.stop, self.stm.reset, self.stm.clear, self.show_help_window])
        add_separator()
        add_control_widget(_DAC_Slider_Control, "Bias", "32768", self.stm.set_bias, stm_control.STM_Status.dac_to_bias_volts, from_=0, to=65535)
        add_control_widget(_DAC_Slider_Control, "DACZ", "32768", self.stm.set_dacz, stm_control.STM_Status.dac_to_dacz_volts, from_=0, to=65535)
        add_control_widget(_DAC_Slider_Control, "DACX", "32768", self.stm.set_dacx, stm_control.STM_Status.dac_to_dacx_volts, from_=0, to=65535)
        add_control_widget(_DAC_Slider_Control, "DACY", "32768", self.stm.set_dacy, stm_control.STM_Status.dac_to_dacy_volts, from_=0, to=65535)
        add_separator()
        add_control_widget(_ButtonWithEntry, "Approach", ["500", "10000", "10"], self.stm.approach, display_list=["Target", "Max Steps", "Step Interval"])
        add_control_widget(MotorControl, stm_control=self.stm)
        add_control_widget(_ButtonWithEntry, "Plot IV", ["0", "65535", "100"], self._plot_iv_curve, display_list=["Start", "End", "Steps"])
        add_control_widget(_ButtonWithEntry, "Save IV", ["./data/iv_curve_"], self._save_iv_curve)
        add_separator()
        add_control_widget(_ButtonWithEntry, "Set PID", ["0.0001", "0.0001", "0.0"], self.stm.set_pid, display_list=["Kp", "Ki", "Kd"])
        const_current_frame = ttk.Frame(self.control_frames)
        const_current_frame.grid(row=row_number, column=0, pady=2, sticky='ew')
        const_current_frame.grid_columnconfigure(1, weight=1)
        self.const_current_button = ttk.Button(const_current_frame, text="ConstCurrent On", command=self.toggle_const_current)
        self.const_current_button.grid(row=0, column=0, sticky='w', padx=(0,5))
        self.const_current_target_var = tk.StringVar(value="1000")
        const_current_entry = ttk.Entry(const_current_frame, textvariable=self.const_current_target_var, width=15)
        const_current_entry.grid(row=0, column=1, sticky='ew')
        row_number += 1
        
        add_separator()

        scan_control_widget = add_control_widget(_ScanControl, self.start_scan_thread)
        self.scan_button = scan_control_widget.scan_button
        add_control_widget(_ButtonWithEntry, "Save Scan", ["./data/image"], self._save_scan_image)
        add_separator()
        
        self.status_label = ttk.Label(self.control_frames, text="No Updates", relief=tk.RAISED, anchor='w', wraplength=380)
        self.status_label.grid(row=row_number, column=0, sticky='ew', pady=(10,0))

    def show_help_window(self):
        help_win = tk.Toplevel(self)
        help_win.title("STM Controller Help")
        help_win.geometry("600x700")

        # --- Create a scrollable frame inside the help window ---
        outer_frame = ttk.Frame(help_win)
        outer_frame.pack(fill='both', expand=True)
        outer_frame.grid_rowconfigure(0, weight=1)
        outer_frame.grid_columnconfigure(0, weight=1)
        canvas = tk.Canvas(outer_frame)
        canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar = ttk.Scrollbar(outer_frame, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky='ns')
        canvas.configure(yscrollcommand=scrollbar.set)
        content_frame = ttk.Frame(canvas, padding=15)
        canvas_frame_id = canvas.create_window((0, 0), window=content_frame, anchor="nw")
        def on_frame_cfg(e): canvas.configure(scrollregion=canvas.bbox("all"))
        def on_canvas_cfg(e): canvas.itemconfig(canvas_frame_id, width=e.width)
        content_frame.bind("<Configure>", on_frame_cfg)
        canvas.bind("<Configure>", on_canvas_cfg)
        
        # --- Populate the help content ---
        row = 0
        def add_help_section(heading, body):
            nonlocal row
            # Heading
            head_style = ttk.Style()
            head_style.configure("Heading.TLabel", font=('Helvetica', 12, 'bold'))
            head_label = ttk.Label(content_frame, text=heading, style="Heading.TLabel")
            head_label.grid(row=row, column=0, sticky='w', pady=(15, 2))
            row += 1
            # Body
            body_label = ttk.Label(content_frame, text=body, wraplength=550, justify=tk.LEFT)
            body_label.grid(row=row, column=0, sticky='w', padx=10, pady=(0, 5))
            row += 1
            # Separator
            ttk.Separator(content_frame, orient='horizontal').grid(row=row, column=0, sticky='ew', pady=10)
            row += 1

        # --- CONTENT ---
        add_help_section("Connection & General",
            "• Open: Connects to the STM hardware via the specified serial (COM) port.\n"
            "• STOP: Immediately halts any active process (like Approach or Scan).\n"
            "• Reset: Sends a reset command to the microcontroller.\n"
            "• Clear: Clears internal data buffers in the controller and plot history in the app.")

        add_help_section("DAC Controls",
            "These controls directly set the raw value (0-65535 for 16-bit) for the Digital-to-Analog Converters, which control voltages. The calculated output voltage is shown to the right.\n"
            "• Bias: Sets the voltage difference between the tip and the sample. This is critical for tunneling.\n"
            "• DACZ: Controls the fine Z-axis (height) of the piezo scanner.\n"
            "• DACX / DACY: Controls the X and Y position of the piezo scanner.")

        add_help_section("Automated Procedures",
            "• Approach: Starts an automated tip approach. The tip moves towards the sample until a target current is detected.\n"
            "   - Target: The ADC value representing the desired tunneling current to stop at.\n"
            "   - Steps: The number of steps for the approach motor.\n\n"
            "• Plot IV: Performs current-voltage (I-V) spectroscopy. It sweeps the bias voltage and measures the resulting current, then plots the result in the 'Curves' tab.\n"
            "   - Start / End: The DAC values for the beginning and end of the bias voltage sweep.\n"
            "   - Steps: The number of data points to measure during the sweep.")

        add_help_section("Feedback and Scanning",
            "• Set PID: Sets the gains for the Z-axis feedback loop, which is essential for constant-current imaging.\n"
            "   - Kp (Proportional): Responds to the current error.\n"
            "   - Ki (Integral): Corrects long-term, steady-state errors.\n"
            "   - Kd (Derivative): Dampens oscillations.\n\n"
            "• ConstCurrent On: Engages the PID feedback loop to maintain a constant tunneling current by adjusting the Z-height.\n"
            "   - Target ADC: The desired current (as a raw ADC value) for the feedback loop to maintain.\n\n"
            "• Scan: Starts a raster scan of the surface. Requires constant current mode to be active.\n"
            "   - X/Y Start / End: Defines the corners of the scan area in DAC units.\n"
            "   - Interval: The step size in DAC units between pixels. Smaller values = higher resolution.\n"
            "   - Samples: The number of ADC readings to average at each pixel to reduce noise.")

        add_help_section("Data Saving",
            "• Save IV: Saves the most recently measured I-V curve to a .csv file.\n"
            "• Save Scan: Saves the most recent scan data. Two text files (.txt) for ADC and Z-DAC values are created, along with two corresponding image files (.png).")

    def toggle_const_current(self):
        # ... (implementation unchanged)
        if self.is_const_current_on:
            self.stm.turn_off_const_current()
            self.const_current_button.configure(style='TButton')
            self.is_const_current_on = False
        else:
            try:
                target_adc = int(self.const_current_target_var.get())
                self.stm.turn_on_const_current(target_adc)
                self.const_current_button.configure(style='Blue.TButton')
                self.is_const_current_on = True
            except (ValueError, TypeError) as e:
                print(f"Invalid ADC target value for constant current: {e}")

    def start_scan_thread(self, *args):
        self.scan_button.configure(style='Blue.TButton', state=tk.DISABLED)
        scan_thread = threading.Thread(target=self._scan_task, args=args, daemon=True)
        scan_thread.start()

    def _scan_task(self, *args):
        try:
            self.stm.start_scan(*args)
        finally:
            self.after(0, self._on_scan_complete)

    def _on_scan_complete(self):
        self.scan_button.configure(style='TButton', state=tk.NORMAL)

    def _quit(self):
        self.quit(); self.destroy()

    def _reset(self):
        self.stm.reset()

    def _update_real_time(self):
        if not self.stm.busy:
            status = self.stm.get_status()
            if self.stm.history:
                self.status_label.config(text=status.to_string())
                plot_x_sec = [(h.time_millis - self.stm.history[-1].time_millis) / 1000.0 for h in self.stm.history]
                plot_adc = [stm_control.STM_Status.adc_to_amp(h.adc) for h in self.stm.history]
                plot_steps = [h.steps for h in self.stm.history]
                self.real_time_current_plot_frame.update_plot(plot_x_sec, plot_adc)
                self.real_time_steps_plot_frame.update_plot(plot_x_sec, plot_steps)
        self.after(200, self._update_real_time)

    def _update_images(self):
        if np.any(self.stm.scan_adc):
            self.scan_adc_frame.update_image(self.stm.scan_adc)
            self.scan_dacz_frame.update_image(self.stm.scan_dacz)
        self.after(200, self._update_images)

    def _plot_iv_curve(self, *args):
        try:
            int_args = [int(arg) for arg in args]
            iv_values = self.stm.measure_iv_curve(*int_args)
            if not iv_values: return
            bias = [stm_control.STM_Status.dac_to_bias_volts(d) for d in iv_values[::2]]
            current = [stm_control.STM_Status.adc_to_amp(a) for a in iv_values[1::2]]
            self.iv_curve_frame.update_plot(bias, current)
        except Exception as e:
            print(f"Error plotting IV curve: {e}")
            
    def _save_iv_curve(self, filename_prefix):
        iv_values = self.stm.get_iv_curve()
        if not iv_values: return print("No IV curve data to save.")
        x = [stm_control.STM_Status.dac_to_bias_volts(d) for d in iv_values[::2]]
        y = [stm_control.STM_Status.adc_to_amp(a) for a in iv_values[1::2]]
        save_data_to_file(filename_prefix, zip(x, y))
        print(f"IV curve data saved with prefix {filename_prefix}")
        
    def _save_scan_image(self, image_path_prefix):
        ts = int(datetime.timestamp(datetime.now())*1000)
        np.savetxt(f"{image_path_prefix}_adc_{ts}.txt", self.stm.scan_adc)
        np.savetxt(f"{image_path_prefix}_dacz_{ts}.txt", self.stm.scan_dacz)
        self.scan_adc_frame.save_figure(f"{image_path_prefix}_adc_{ts}.png")
        self.scan_dacz_frame.save_figure(f"{image_path_prefix}_dacz_{ts}.png")
        print(f"Scan images and data saved with prefix {image_path_prefix}")

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app._quit)
    app.mainloop()