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

    def add_plot(self, label=None, xlabel=None, ylabel=None, ylabel2=None):
        self.ax = self.figure.add_subplot(111)
        self.plot = self.ax.plot([0], [0], '-', label=label, color='tab:blue')[0]
        self.ax.grid(True)
        self.ax.tick_params(axis='y', labelcolor='tab:blue')
        if xlabel:
            self.ax.set(xlabel=xlabel)
        if ylabel:
            self.ax.set(ylabel=ylabel)
            self.ax.yaxis.label.set_color('tab:blue')
        
        # Add secondary y-axis if requested (for same data in different units)
        self.ax2 = None
        self.has_dual_axis = False
        if ylabel2:
            self.ax2 = self.ax.twinx()
            self.ax2.set_ylabel(ylabel2, color='tab:orange')
            self.ax2.tick_params(axis='y', labelcolor='tab:orange')
            self.has_dual_axis = True
            self.ax.legend(loc='upper right')
        else:
            self.ax.legend(loc='upper right')

    def add_image(self, image, title=None):
        self.ax = self.figure.add_subplot(111)
        if title:
            self.ax.set_title(title)
        
        self.image = self.ax.imshow(image, interpolation='nearest', norm='linear', origin="lower", aspect='equal', cmap='jet')
        self.cbar = self.figure.colorbar(self.image, ax=self.ax, fraction=0.046, pad=0.04)


    def update_plot(self, x_data, y_data, conversion_func=None):
        if not hasattr(self, 'plot'): return
        self.plot.set_xdata(x_data)
        self.plot.set_ydata(y_data)
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Update secondary axis limits if it exists (showing same data in different units)
        if self.has_dual_axis and self.ax2 is not None and len(y_data) > 0:
            y_min, y_max = self.ax.get_ylim()
            
            # Use provided conversion function or default to ADC conversion
            if conversion_func:
                converted_min = conversion_func(y_min)
                converted_max = conversion_func(y_max)
            else:
                # Default: ADC conversion for current plot
                converted_min = stm_control.STM_Status.amp_to_adc(y_min)
                converted_max = stm_control.STM_Status.amp_to_adc(y_max)
            
            self.ax2.set_ylim(converted_min, converted_max)
        
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


class CollapsibleSection(ttk.Frame):
    """A collapsible section widget with modern styling that adapts to system theme."""
    def __init__(self, parent, title, *args, **kwargs):
        super().__init__(parent, relief=tk.FLAT, borderwidth=0, *args, **kwargs)
        self.is_expanded = True
        
        # Use ttk widgets for automatic theme support
        self.header = ttk.Frame(self, relief=tk.RAISED, borderwidth=1)
        self.header.pack(fill=tk.X, pady=(0, 1))
        self.header.bind('<Button-1>', self.toggle)
        
        # Arrow indicator using ttk
        self.arrow_label = ttk.Label(self.header, text="▼", width=2, font=('Segoe UI', 10), cursor='hand2')
        self.arrow_label.pack(side=tk.LEFT, padx=(8, 0))
        self.arrow_label.bind('<Button-1>', self.toggle)
        
        # Title label using ttk
        self.title_label = ttk.Label(self.header, text=title, font=('Segoe UI', 10, 'bold'), cursor='hand2')
        self.title_label.pack(side=tk.LEFT, padx=8, pady=8)
        self.title_label.bind('<Button-1>', self.toggle)
        
        # Content frame using ttk for theme support
        self.content = ttk.Frame(self, relief=tk.FLAT, borderwidth=1)
        self.content.pack(fill=tk.BOTH, expand=True, padx=1, pady=(0, 2))
        
    def toggle(self, event=None):
        """Toggle the section expanded/collapsed state."""
        if self.is_expanded:
            self.content.pack_forget()
            self.arrow_label.config(text="▶")
            self.is_expanded = False
        else:
            self.content.pack(fill=tk.BOTH, expand=True, padx=1, pady=(0, 2))
            self.arrow_label.config(text="▼")
            self.is_expanded = True
    
    def get_content_frame(self):
        """Get the content frame to add widgets to."""
        return self.content

class ApproachAndMotorControl(ttk.Frame):
    def __init__(self, parent, stm_control, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.stm = stm_control
        self.grid_columnconfigure(2, weight=1) # Allow entry column to expand
        self.grid_columnconfigure(3, weight=0) # Current display column

        # --- Buttons ---
        button_frame = ttk.Frame(self)
        button_frame.grid(row=0, column=0, rowspan=5, sticky='ns', padx=(0, 10))
        button_frame.grid_rowconfigure(0, weight=1)
        button_frame.grid_rowconfigure(1, weight=1)
        button_frame.grid_rowconfigure(2, weight=1)
        button_frame.grid_rowconfigure(3, weight=1)
        button_frame.grid_rowconfigure(4, weight=1)

        approach_button = ttk.Button(button_frame, text="Approach", command=self._start_approach)
        approach_button.grid(row=0, column=0, sticky='ew', pady=2)

        back_button = ttk.Button(button_frame, text="Back", command=self._move_backward)
        back_button.grid(row=1, column=0, sticky='ew', pady=2)

        forward_button = ttk.Button(button_frame, text="Forward", command=self._move_forward)
        forward_button.grid(row=2, column=0, sticky='ew', pady=2)

        stop_button = ttk.Button(button_frame, text="Stop", command=self.stm.stepper_stop)
        stop_button.grid(row=3, column=0, sticky='ew', pady=2)

        # --- Labels and Entries ---
        ttk.Label(self, text="Target (ADC):").grid(row=0, column=1, sticky='w')
        self.target_var = tk.StringVar(value="500")
        target_entry = ttk.Entry(self, textvariable=self.target_var, width=10)
        target_entry.grid(row=0, column=2, sticky='ew', pady=2)
        target_entry.bind('<KeyRelease>', self._update_target_display)
        
        # Display equivalent current
        self.target_current_label = ttk.Label(self, text="≈ 0.00 nA", font=('TkDefaultFont', 8), foreground='gray')
        self.target_current_label.grid(row=0, column=3, sticky='w', padx=(5, 0))

        ttk.Label(self, text="Max Steps:").grid(row=1, column=1, sticky='w')
        self.max_steps_var = tk.StringVar(value="10000")
        max_steps_entry = ttk.Entry(self, textvariable=self.max_steps_var, width=10)
        max_steps_entry.grid(row=1, column=2, sticky='ew', pady=2)
        max_steps_entry.bind('<KeyRelease>', self._update_max_steps_display)

        # Display equivalent distance for max steps
        self.max_steps_distance_label = ttk.Label(self, text="≈ 0.00 μm", font=('TkDefaultFont', 8), foreground='gray')
        self.max_steps_distance_label.grid(row=1, column=3, sticky='w', padx=(5, 0))

        ttk.Label(self, text="Steps:").grid(row=2, column=1, sticky='w')
        self.steps_var = tk.StringVar(value="10")
        steps_entry = ttk.Entry(self, textvariable=self.steps_var, width=10)
        steps_entry.grid(row=2, column=2, sticky='ew', pady=2)
        steps_entry.bind('<KeyRelease>', self._update_steps_display)
        
        # Display equivalent distance
        self.steps_distance_label = ttk.Label(self, text="≈ 0.00 μm", font=('TkDefaultFont', 8), foreground='gray')
        self.steps_distance_label.grid(row=2, column=3, sticky='w', padx=(5, 0))

        ttk.Label(self, text="Direction:").grid(row=3, column=1, sticky='w')
        self.direction_var = tk.StringVar(value="Forward")
        direction_combo = ttk.Combobox(self, textvariable=self.direction_var, 
                                     values=["Forward", "Backward"], 
                                     state="readonly", width=8)
        direction_combo.grid(row=3, column=2, sticky='ew', pady=2)
        
        # Add Fine/Coarse mode controls with radio buttons
        ttk.Label(self, text="Mode:").grid(row=4, column=1, sticky='w')
        mode_frame = ttk.Frame(self)
        mode_frame.grid(row=4, column=2, columnspan=2, sticky='ew', pady=2)
        
        self.mode_var = tk.StringVar(value="Coarse")
        coarse_radio = ttk.Radiobutton(mode_frame, text="Coarse", variable=self.mode_var, 
                                       value="Coarse", command=self._set_coarse_mode)
        coarse_radio.pack(side=tk.LEFT, padx=(0, 10))
        
        fine_radio = ttk.Radiobutton(mode_frame, text="Fine", variable=self.mode_var, 
                                     value="Fine", command=self._set_fine_mode)
        fine_radio.pack(side=tk.LEFT)
        
        # Mode info label
        mode_info = ttk.Label(self, text="Coarse: use motor steps | Fine: 1 motor step + piezo sweep", 
                              font=('TkDefaultFont', 8), foreground='gray')
        mode_info.grid(row=5, column=1, columnspan=3, sticky='w', pady=(0, 5))
        
        # Add info label
        info_label = ttk.Label(self, text="Forward: tip towards sample, Backward: tip away from sample", 
                              font=('TkDefaultFont', 8), foreground='gray')
        info_label.grid(row=6, column=1, columnspan=3, sticky='w', pady=(0, 5))
        
        # Add approach status indicator (for recovery mode, etc.)
        self.approach_status_label = ttk.Label(self, text="", font=('TkDefaultFont', 9, 'bold'))
        self.approach_status_label.grid(row=7, column=1, columnspan=3, sticky='w', pady=(5, 5))
        
        # Initialize displays
        self._update_target_display()
        self._update_max_steps_display()
        self._update_steps_display()



    def _update_target_display(self, event=None):
        """Update the current display when target ADC value changes."""
        try:
            target_adc = int(self.target_var.get())
            current_amps = stm_control.STM_Status.adc_to_amp(target_adc)
            current_nanoamps = current_amps * 1e9  # Convert to nA
            self.target_current_label.config(text=f"≈ {current_nanoamps:.2f} nA")
        except (ValueError, AttributeError):
            self.target_current_label.config(text="≈ --- nA")
    
    def _update_max_steps_display(self, event=None):
        """Update the distance display when max steps value changes."""
        try:
            steps = int(self.max_steps_var.get())
            distance_um = stm_control.STM_Status.steps_to_distance(steps)
            if abs(distance_um) >= 1000:
                distance_mm = distance_um / 1000
                self.max_steps_distance_label.config(text=f"≈ {distance_mm:.3f} mm")
            else:
                self.max_steps_distance_label.config(text=f"≈ {distance_um:.2f} μm")
        except (ValueError, AttributeError):
            self.max_steps_distance_label.config(text="≈ --- μm")

    def _update_steps_display(self, event=None):
        """Update the distance display when steps value changes."""
        try:
            steps = int(self.steps_var.get())
            distance_um = stm_control.STM_Status.steps_to_distance(steps)
            if abs(distance_um) >= 1000:
                distance_mm = distance_um / 1000
                self.steps_distance_label.config(text=f"≈ {distance_mm:.3f} mm")
            else:
                self.steps_distance_label.config(text=f"≈ {distance_um:.2f} μm")
        except (ValueError, AttributeError):
            self.steps_distance_label.config(text="≈ --- μm")
    
    def _set_coarse_mode(self):
        """Set approach to coarse mode (use motor steps)."""
        if self.stm.is_opened:
            self.stm.disable_fine_motor_mode()
            print("Approach mode: COARSE (motor steps)")
    
    def _set_fine_mode(self):
        """Set approach to fine mode (1 motor step + piezo sweep)."""
        if self.stm.is_opened:
            self.stm.enable_fine_motor_mode()
            print("Approach mode: FINE (1 motor step + piezo sweep)")
    
    def _start_approach(self):
        try:
            target_adc = int(self.target_var.get())
            max_steps = int(self.max_steps_var.get())
            step_interval = int(self.steps_var.get())
            direction = 1 if self.direction_var.get() == "Forward" else -1
            self.stm.approach(target_adc, max_steps, step_interval, direction)
            current_amps = stm_control.STM_Status.adc_to_amp(target_adc)
            current_nanoamps = current_amps * 1e9
            print(f"Starting approach in {self.direction_var.get()} direction")
            print(f"Target: {target_adc} ADC (≈ {current_nanoamps:.2f} nA)")
        except ValueError:
            print("Invalid approach parameters")

    def _move_backward(self):
        try:
            steps = int(self.steps_var.get())
            distance_um = stm_control.STM_Status.steps_to_distance(steps)
            self.stm.move_motor(-steps)
            print(f"Moving backward: {steps} steps ({distance_um:.2f} μm)")
            # Force immediate GUI update after motor move
            self.after_idle(lambda: self.stm.get_status())
        except ValueError:
            self.steps_var.set("10")

    def _move_forward(self):
        try:
            steps = int(self.steps_var.get())
            distance_um = stm_control.STM_Status.steps_to_distance(steps)
            self.stm.move_motor(steps)
            print(f"Moving forward: {steps} steps ({distance_um:.2f} μm)")
            # Force immediate GUI update after motor move
            self.after_idle(lambda: self.stm.get_status())
        except ValueError:
            self.steps_var.set("10")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM Controller")
        self.geometry("1200x800")
        self.after_id = None
        self.after_id_images = None
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.wm_title("Panda STM")
        self.is_const_current_on = False

        self.stm = stm_control.STM()
        
        # Configure modern styles that work with system theme
        self.style = ttk.Style(self)
        
        # Try to use native theme if available
        try:
            # On macOS, use 'aqua' theme which supports dark mode
            available_themes = self.style.theme_names()
            if 'aqua' in available_themes:
                self.style.theme_use('aqua')
            elif 'clam' in available_themes:
                self.style.theme_use('clam')
        except:
            pass  # Use default theme
        
        # Blue button style for active operations (adapts to theme)
        self.style.configure('Blue.TButton')
        self.style.map('Blue.TButton',
            background=[('active', '#00599c'), ('!disabled', '#0078d4')],
            foreground=[('active', 'white'), ('!disabled', 'white')]
        )

        # Create status bar at the bottom with theme support
        self.status_bar = ttk.Frame(self, relief=tk.SUNKEN, borderwidth=1)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Status bar sections - using ttk for theme support
        self.status_connection = ttk.Label(self.status_bar, text="⚫ Disconnected", 
                                          relief=tk.FLAT, anchor=tk.W, padding=(5, 2))
        self.status_connection.pack(side=tk.LEFT, padx=2)
        
        ttk.Separator(self.status_bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=2)
        
        self.status_mode = ttk.Label(self.status_bar, text="Mode: Idle", 
                                    relief=tk.FLAT, anchor=tk.W, padding=(5, 2))
        self.status_mode.pack(side=tk.LEFT, padx=2)
        
        ttk.Separator(self.status_bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=2)
        
        self.status_current = ttk.Label(self.status_bar, text="Current: --- nA", 
                                       relief=tk.FLAT, anchor=tk.W, padding=(5, 2))
        self.status_current.pack(side=tk.LEFT, padx=2)
        
        ttk.Separator(self.status_bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=2)
        
        self.status_position = ttk.Label(self.status_bar, text="Steps: 0", 
                                        relief=tk.FLAT, anchor=tk.W, padding=(5, 2))
        self.status_position.pack(side=tk.LEFT, padx=2)
        
        ttk.Separator(self.status_bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=2)
        
        self.status_message = ttk.Label(self.status_bar, text="Ready", 
                                       relief=tk.FLAT, anchor=tk.W, padding=(5, 2))
        self.status_message.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

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
        self.real_time_current_plot_frame.add_plot(label="Current (A)", xlabel='Time (s)', ylabel='Current (A)', ylabel2='ADC Value')
        self.real_time_current_plot_frame.grid(row=0, column=0, padx=5, pady=5, sticky='nsew')

        self.real_time_steps_plot_frame = PlotFrame(curve_tab_frame, with_toobar=True)
        self.real_time_steps_plot_frame.add_plot(label="Motor Position", xlabel='Time (s)', ylabel='Steps', ylabel2='Distance (μm)')
        self.real_time_steps_plot_frame.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        self.iv_curve_frame = PlotFrame(curve_tab_frame, with_toobar=True)
        self.iv_curve_frame.add_plot(label="IV Curve", xlabel="Bias (V)", ylabel="Current (A)")
        self.iv_curve_frame.grid(row=0, column=1, padx=5, pady=5, sticky='nsew')

        # --- Place plots into the "Image" tab ---
        self.scan_dacz_frame = PlotFrame(image_tab_frame, with_toobar=True)
        self.scan_dacz_frame.add_image(np.zeros((10, 10)), title="DAC Z Output (Topography)")
        self.scan_dacz_frame.grid(row=0, column=0, padx=5, pady=5, sticky='nsew') # row 0, col 0
        
        self.scan_adc_frame = PlotFrame(image_tab_frame, with_toobar=True)
        self.scan_adc_frame.add_image(np.zeros((10, 10)), title="ADC Reading (Tunnelling Current)")
        self.scan_adc_frame.grid(row=0, column=1, padx=5, pady=5, sticky='nsew') # row 0, col 1


        self.setup_control_widgets()
        self._update_images()
        self._update_real_time()
        self.state('zoomed')
        self.after_idle(lambda: main_pane.sashpos(0, 400))

    def setup_control_widgets(self):
        self.control_frames.grid_columnconfigure(0, weight=1)
        row_number = 0
        
        def add_collapsible_section(title):
            """Add a collapsible section and return its content frame."""
            nonlocal row_number
            section = CollapsibleSection(self.control_frames, title)
            section.grid(row=row_number, column=0, sticky='ew', pady=2)
            row_number += 1
            return section.get_content_frame()

        class _DAC_Slider_Control(ttk.Frame):
            def __init__(self, parent, text, default_value, cmd_func, convert_func, from_, to, *args, **kwargs):
                super().__init__(parent, *args, **kwargs)
                self.grid_columnconfigure(1, weight=1)
                self.cmd_func, self.convert_func = cmd_func, convert_func
                self.from_ = from_
                self.to = to
                
                # Convert default_value to int if it's a string
                if isinstance(default_value, str):
                    default_value = int(default_value)
                
                # Shift the range to be 0-based to avoid ttk.Scale rendering issues with negative values
                self.range_offset = -from_  # Offset to shift range to start at 0
                self.shifted_default = default_value + self.range_offset
                
                # Use DoubleVar for smooth slider movement
                self.variable = tk.DoubleVar(value=float(self.shifted_default))

                self.label = ttk.Label(self, text=text, width=5)
                self.label.grid(row=0, column=0, sticky=tk.W)

                # Create slider with 0-based range (shifted)
                self.slider = ttk.Scale(self, from_=0.0, to=float(to - from_), orient=tk.HORIZONTAL, variable=self.variable, command=self._on_slider_change)
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
                # Convert from shifted (0-based) value back to actual DAC value
                shifted_value = int(float(value_str))
                actual_value = shifted_value - self.range_offset
                self.input_string_var.set(str(actual_value))
                self._update_display(actual_value)
                self.cmd_func(actual_value)

            def _on_entry_set(self, event=None):
                try:
                    value = int(self.input_string_var.get())
                    # Clamp value to valid range
                    if value < self.from_: 
                        value = self.from_
                    if value > self.to: 
                        value = self.to
                    # Convert to shifted value for slider
                    shifted_value = value + self.range_offset
                    self.variable.set(float(shifted_value))
                    self.input_string_var.set(str(value))
                    self._update_display(value)
                    self.cmd_func(value)
                except (ValueError, TypeError):
                    # Get current shifted value and convert back to actual
                    shifted_val = int(round(self.variable.get()))
                    current_val = shifted_val - self.range_offset
                    self.input_string_var.set(str(current_val))

            def _update_display(self, value):
                try:
                    voltage = self.convert_func(int(value))
                    self.display_var.set(f"{voltage:.4f} V")
                except (ValueError, TypeError):
                    self.display_var.set("Invalid")

        class _DACControlWithScaling(ttk.Frame):
            def __init__(self, parent, label, stm, set_dac_func, dac_to_volts_func, set_scaling_func, get_scaling_func, *args, **kwargs):
                super().__init__(parent, *args, **kwargs)
                self.stm = stm
                self.set_scaling_func = set_scaling_func
                self.get_scaling_func = get_scaling_func
                self.grid_columnconfigure(1, weight=1)

                self.slider_control = _DAC_Slider_Control(self, label, 0, set_dac_func, dac_to_volts_func, from_=-32768, to=32767)
                self.slider_control.pack(fill='x', expand=True)

                scale_frame = ttk.Frame(self)
                scale_frame.pack(fill='x', expand=True, pady=(5,0))
                scale_frame.grid_columnconfigure(1, weight=1)

                ttk.Label(scale_frame, text="Scale Factor:").grid(row=0, column=0, sticky='w')
                self.scale_var = tk.StringVar(value=str(get_scaling_func()))
                self.scale_entry = ttk.Entry(scale_frame, textvariable=self.scale_var, width=10)
                self.scale_entry.grid(row=0, column=1, sticky='ew', padx=5)
                self.scale_entry.bind('<Return>', self._on_scale_set)
                self.scale_entry.bind('<FocusOut>', self._on_scale_set)

            def _on_scale_set(self, event=None):
                try:
                    factor = float(self.scale_var.get())
                    self.set_scaling_func(factor)
                    # Update the display to reflect the new scale
                    current_slider_value = self.slider_control.variable.get()
                    self.slider_control._update_display(current_slider_value)
                except ValueError:
                    self.scale_var.set(str(self.get_scaling_func()))

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
                # Bipolar defaults: scan centered around 0 (±1000 DAC units ≈ ±230 nm)
                labels = ["Start", "End", "Interval"]; defaults = [["-1000", "1000", "256"], ["-1000", "1000", "256"]]
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

        # ===== CONNECTION SECTION =====
        connection_section = add_collapsible_section("🔌 Connection")
        
        connection_frame = ttk.Frame(connection_section)
        connection_frame.pack(fill=tk.X, pady=5, padx=5)
        connection_frame.grid_columnconfigure(1, weight=1)

        open_button = ttk.Button(connection_frame, text="Open", command=self._open_connection)
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
        
        control_buttons = _MultipleButtons(connection_section, ["STOP", "Reset", "Clear", "Help"],
                           [self._stop_operation, self._reset_stm, self._clear_data, self.show_help_window])
        control_buttons.pack(fill=tk.X, pady=5, padx=5)
        
        # ===== DAC CONTROLS SECTION =====
        dac_section = add_collapsible_section("⚡ DAC Controls")
        
        _DACControlWithScaling(dac_section, label="Bias", stm=self.stm, set_dac_func=self.stm.set_bias, 
                              dac_to_volts_func=stm_control.STM_Status.dac_to_bias_volts, 
                              set_scaling_func=stm_control.STM_Status.set_bias_scaling_factor, 
                              get_scaling_func=lambda: stm_control.STM_Status.bias_scaling_factor).pack(fill=tk.X, pady=2, padx=5)
        
        _DACControlWithScaling(dac_section, label="DACZ", stm=self.stm, set_dac_func=self.stm.set_dacz, 
                              dac_to_volts_func=stm_control.STM_Status.dac_to_dacz_volts, 
                              set_scaling_func=stm_control.STM_Status.set_z_scaling_factor, 
                              get_scaling_func=lambda: stm_control.STM_Status.z_scaling_factor).pack(fill=tk.X, pady=2, padx=5)
        
        _DACControlWithScaling(dac_section, label="DACX", stm=self.stm, set_dac_func=self.stm.set_dacx, 
                              dac_to_volts_func=stm_control.STM_Status.dac_to_dacx_volts, 
                              set_scaling_func=stm_control.STM_Status.set_x_scaling_factor, 
                              get_scaling_func=lambda: stm_control.STM_Status.x_scaling_factor).pack(fill=tk.X, pady=2, padx=5)
        
        _DACControlWithScaling(dac_section, label="DACY", stm=self.stm, set_dac_func=self.stm.set_dacy, 
                              dac_to_volts_func=stm_control.STM_Status.dac_to_dacy_volts, 
                              set_scaling_func=stm_control.STM_Status.set_y_scaling_factor, 
                              get_scaling_func=lambda: stm_control.STM_Status.y_scaling_factor).pack(fill=tk.X, pady=2, padx=5)
        
        # ===== APPROACH & MOTOR SECTION =====
        approach_section = add_collapsible_section("🎯 Approach & Motor")
        self.approach_control = ApproachAndMotorControl(approach_section, stm_control=self.stm)
        self.approach_control.pack(fill=tk.X, pady=5, padx=5)
        
        # Add Fine Approach Parameters
        fine_params_frame = ttk.Frame(approach_section)
        fine_params_frame.pack(fill=tk.X, pady=5, padx=5)
        fine_params_frame.grid_columnconfigure(1, weight=1)
        
        ttk.Label(fine_params_frame, text="Fine Z Step:").grid(row=0, column=0, sticky='w', padx=(0, 5))
        self.fine_step_var = tk.StringVar(value="10")
        fine_step_entry = ttk.Entry(fine_params_frame, textvariable=self.fine_step_var, width=10)
        fine_step_entry.grid(row=0, column=1, sticky='ew', padx=(0, 5))
        ttk.Button(fine_params_frame, text="Set", command=self._set_fine_step_size).grid(row=0, column=2)
        
        ttk.Label(fine_params_frame, text="Z Range:").grid(row=1, column=0, sticky='w', padx=(0, 5), pady=(5, 0))
        self.z_range_var = tk.StringVar(value="2000")
        z_range_entry = ttk.Entry(fine_params_frame, textvariable=self.z_range_var, width=10)
        z_range_entry.grid(row=1, column=1, sticky='ew', padx=(0, 5), pady=(5, 0))
        z_range_entry.bind('<KeyRelease>', self._update_z_range_display)
        ttk.Button(fine_params_frame, text="Set", command=self._set_z_range).grid(row=1, column=2, pady=(5, 0))
        
        # Display equivalent distance for Z range
        self.z_range_distance_label = ttk.Label(fine_params_frame, text="≈ 0.00 nm", font=('TkDefaultFont', 8), foreground='gray')
        self.z_range_distance_label.grid(row=1, column=3, sticky='w', padx=(5, 0), pady=(5, 0))
        
        # Initialize Z range display
        self._update_z_range_display()
        
        # Info label for fine parameters
        fine_info = ttk.Label(approach_section, text="Fine Z Step: piezo step size | Z Range: sweep search range", 
                             font=('TkDefaultFont', 8), foreground='gray')
        fine_info.pack(fill=tk.X, pady=(0, 5), padx=5)
        
        # Add Approach Debug button
        ttk.Button(approach_section, text="Approach Debug", command=self._show_approach_debug).pack(fill=tk.X, pady=2, padx=5)
        
        # ===== SPECTROSCOPY SECTION =====
        spectroscopy_section = add_collapsible_section("📊 Spectroscopy")
        
        _ButtonWithEntry(spectroscopy_section, "Plot IV", ["0", "65535", "100"], self._plot_iv_curve, 
                        display_list=["Start", "End", "Steps"]).pack(fill=tk.X, pady=2, padx=5)
        _ButtonWithEntry(spectroscopy_section, "Save IV", ["./data/iv_curve_"], self._save_iv_curve).pack(fill=tk.X, pady=2, padx=5)
        
        # ===== FEEDBACK CONTROL SECTION =====
        feedback_section = add_collapsible_section("🔄 Feedback Control")
        
        _ButtonWithEntry(feedback_section, "Set PID", ["0.0001", "0.0001", "0.0"], self.stm.set_pid, 
                        display_list=["Kp", "Ki", "Kd"]).pack(fill=tk.X, pady=2, padx=5)
        
        # Separate buttons with clearer labels
        ttk.Button(feedback_section, text="📊 Monitor PID (Real-time)", 
                  command=self._show_pid_debug).pack(fill=tk.X, pady=2, padx=5)
        ttk.Button(feedback_section, text="📖 PID Tuning Guide (Help)", 
                  command=self._show_pid_tuning_guide).pack(fill=tk.X, pady=2, padx=5)
        const_current_frame = ttk.Frame(feedback_section)
        const_current_frame.pack(fill=tk.X, pady=5, padx=5)
        const_current_frame.grid_columnconfigure(1, weight=1)
        
        self.const_current_button = ttk.Button(const_current_frame, text="ConstCurrent On", command=self.toggle_const_current)
        self.const_current_button.grid(row=0, column=0, sticky='w', padx=(0,5))
        
        entry_frame = ttk.Frame(const_current_frame)
        entry_frame.grid(row=0, column=1, sticky='ew')
        entry_frame.grid_columnconfigure(0, weight=1)
        
        ttk.Label(entry_frame, text="Target (ADC):", font=('TkDefaultFont', 8)).grid(row=0, column=0, sticky='w')
        self.const_current_target_var = tk.StringVar(value="1000")
        const_current_entry = ttk.Entry(entry_frame, textvariable=self.const_current_target_var, width=10)
        const_current_entry.grid(row=1, column=0, sticky='ew')
        const_current_entry.bind('<KeyRelease>', self._update_cc_target_display)
        
        self.cc_target_current_label = ttk.Label(entry_frame, text="≈ 0.00 nA", font=('TkDefaultFont', 8), foreground='gray')
        self.cc_target_current_label.grid(row=1, column=1, sticky='w', padx=(5, 0))
        
        # Initialize display
        self._update_cc_target_display()
        
        # ===== SCANNING SECTION =====
        scanning_section = add_collapsible_section("🔍 Scanning")
        
        scan_control_widget = _ScanControl(scanning_section, self.start_scan_thread)
        scan_control_widget.pack(fill=tk.X, pady=5, padx=5)
        self.scan_button = scan_control_widget.scan_button
        
        _ButtonWithEntry(scanning_section, "Save Scan", ["./data/image"], self._save_scan_image).pack(fill=tk.X, pady=2, padx=5)
        
        # ===== STATUS DISPLAY SECTION =====
        status_section = add_collapsible_section("📋 Status")
        
        self.status_label = ttk.Label(status_section, text="No Updates", relief=tk.RAISED, anchor='w', wraplength=380)
        self.status_label.pack(fill=tk.X, pady=5, padx=5)


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

        add_help_section("PID Controller Tuning",
            "The PID controller maintains constant tunneling current by adjusting Z-height.\n\n"
            "📐 PID Equation:\n"
            "   Output = Kp×error + Ki×∫error·dt + Kd×(d(error)/dt)\n\n"
            "🎯 Parameter Functions:\n"
            "• Kp (Proportional Gain):\n"
            "   - Responds immediately to current error\n"
            "   - Higher Kp → Faster response, but may overshoot\n"
            "   - Too high → Oscillations and instability\n"
            "   - Typical range: 0.0001 - 0.01\n\n"
            "• Ki (Integral Gain):\n"
            "   - Eliminates steady-state error over time\n"
            "   - Accumulates error: ∫(setpoint - measured)dt\n"
            "   - Higher Ki → Faster elimination of offset\n"
            "   - Too high → Overshoot and slow oscillations\n"
            "   - Typical range: 0.00001 - 0.001\n\n"
            "• Kd (Derivative Gain):\n"
            "   - Predicts future error based on rate of change\n"
            "   - Dampens oscillations and improves stability\n"
            "   - Higher Kd → More damping, slower response\n"
            "   - Too high → Amplifies noise, sluggish response\n"
            "   - Typical range: 0.0 - 0.0001\n\n"
            "🔧 Tuning Strategy (Ziegler-Nichols Method):\n"
            "1. Set Ki=0, Kd=0. Increase Kp until oscillation starts (Kp_critical)\n"
            "2. Measure oscillation period T_critical\n"
            "3. Calculate:\n"
            "   Kp = 0.6 × Kp_critical\n"
            "   Ki = 2 × Kp / T_critical\n"
            "   Kd = Kp × T_critical / 8\n\n"
            "💡 Quick Start Values:\n"
            "   Kp = 0.0001, Ki = 0.0001, Kd = 0.0\n"
            "   (Adjust based on system response)")

        add_help_section("Constant Current & Scanning",
            "• ConstCurrent On: Engages the PID feedback loop to maintain constant tunneling current.\n"
            "   - Target ADC: The desired current (as a raw ADC value) for the feedback loop to maintain.\n"
            "   - The system adjusts Z-height to keep current constant as tip scans over surface features.\n\n"
            "• Scan: Starts a raster scan of the surface. Requires constant current mode to be active.\n"
            "   - X/Y Start / End: Defines the corners of the scan area in DAC units.\n"
            "   - Interval: The step size in DAC units between pixels. Smaller values = higher resolution.\n"
            "   - Samples: The number of ADC readings to average at each pixel to reduce noise.\n\n"
            "📊 Scan Tips:\n"
            "   - Start with small scan areas (100×100 DAC units)\n"
            "   - Use 10-20 samples per pixel for good signal-to-noise ratio\n"
            "   - Ensure PID is well-tuned before scanning\n"
            "   - Monitor current stability during scan")

        add_help_section("Data Saving",
            "• Save IV: Saves the most recently measured I-V curve to a .csv file.\n"
            "• Save Scan: Saves the most recent scan data. Two text files (.txt) for ADC and Z-DAC values are created, along with two corresponding image files (.png).")

    def _update_cc_target_display(self, event=None):
        """Update the current display when constant current target ADC value changes."""
        try:
            target_adc = int(self.const_current_target_var.get())
            current_amps = stm_control.STM_Status.adc_to_amp(target_adc)
            current_nanoamps = current_amps * 1e9  # Convert to nA
            self.cc_target_current_label.config(text=f"≈ {current_nanoamps:.2f} nA")
        except (ValueError, AttributeError):
            self.cc_target_current_label.config(text="≈ --- nA")
    
    def _open_connection(self):
        """Open serial connection and update status bar."""
        try:
            port = self.port_var.get()
            if port:
                self.stm.open(port)
                self.status_connection.config(text="🟢 Connected", foreground='green')
                self.status_message.config(text=f"Connected to {port}", foreground='green')
                print(f"Connected to {port}")
            else:
                self.status_message.config(text="No port selected", foreground='red')
                print("Error: No port selected")
        except Exception as e:
            self.status_connection.config(text="⚫ Connection Failed", foreground='red')
            self.status_message.config(text=f"Connection failed: {str(e)}", foreground='red')
            print(f"Connection error: {e}")
    
    def toggle_const_current(self):
        # ... (implementation unchanged)
        if self.is_const_current_on:
            self.stm.turn_off_const_current()
            self.const_current_button.configure(style='TButton')
            self.is_const_current_on = False
            self.status_message.config(text="Constant current OFF", foreground='black')
        else:
            try:
                target_adc = int(self.const_current_target_var.get())
                self.stm.turn_on_const_current(target_adc)
                self.const_current_button.configure(style='Blue.TButton')
                self.is_const_current_on = True
                current_amps = stm_control.STM_Status.adc_to_amp(target_adc)
                current_nanoamps = current_amps * 1e9
                print(f"Constant current ON - Target: {target_adc} ADC (≈ {current_nanoamps:.2f} nA)")
                self.status_message.config(text=f"Constant current ON - Target: {current_nanoamps:.2f} nA", foreground='green')
            except (ValueError, TypeError) as e:
                print(f"Invalid ADC target value for constant current: {e}")
                self.status_message.config(text="Invalid constant current parameters", foreground='red')

    def start_scan_thread(self, *args):
        self.scan_button.configure(style='Blue.TButton', state=tk.DISABLED)
        self.status_message.config(text="Scan started...", foreground='purple')
        scan_thread = threading.Thread(target=self._scan_task, args=args, daemon=True)
        scan_thread.start()

    def _scan_task(self, *args):
        try:
            self.stm.start_scan(*args)
        finally:
            self.after(0, self._on_scan_complete)

    def _on_scan_complete(self):
        self.scan_button.configure(style='TButton', state=tk.NORMAL)
        self.status_message.config(text="Scan completed", foreground='green')

    def _stop_operation(self):
        """Stop any ongoing operation and update status bar."""
        self.stm.stop()
        self.status_message.config(text="Operation stopped", foreground='red')
        print("Operation stopped")
    
    def _reset_stm(self):
        """Reset STM and update status bar."""
        self.stm.reset()
        self.status_message.config(text="STM reset", foreground='orange')
        print("STM reset")
    
    def _clear_data(self):
        """Clear data history and update status bar."""
        self.stm.clear()
        self.status_message.config(text="Data cleared", foreground='black')
        print("Data cleared")
    
    def _quit(self):
        self.quit(); self.destroy()

    def _reset(self):
        self.stm.reset()

    def _update_real_time(self):
        if not self.stm.busy:
            status = self.stm.get_status()
            if self.stm.history and len(self.stm.history) > 0:
                self.status_label.config(text=status.to_string())
                
                # Update status bar
                self._update_status_bar(status)
                
                # Only update plots if we have new data
                if len(self.stm.history) > 1:
                    plot_x_sec = [(h.time_millis - self.stm.history[-1].time_millis) / 1000.0 for h in self.stm.history]
                    plot_current = [stm_control.STM_Status.adc_to_amp(h.adc) for h in self.stm.history]
                    plot_steps = [h.steps for h in self.stm.history]
                    # Plot current data - secondary axis will show ADC automatically
                    self.real_time_current_plot_frame.update_plot(plot_x_sec, plot_current)
                    # Plot steps data - secondary axis will show distance in μm
                    self.real_time_steps_plot_frame.update_plot(plot_x_sec, plot_steps, 
                                                               stm_control.STM_Status.steps_to_distance)
        self.after_id = self.after(50, self._update_real_time)  # Faster update rate for better responsiveness

    def _update_status_bar(self, status):
        """Update the status bar with current STM status."""
        # Connection status
        if self.stm.is_opened:
            self.status_connection.config(text="🟢 Connected", foreground='green')
        else:
            self.status_connection.config(text="⚫ Disconnected", foreground='gray')
        
        # Mode status
        mode_text = "Mode: "
        mode_color = 'black'
        if status.is_approaching:
            if status.approach_recovery:
                mode_text += "Approaching (Recovery)"
                mode_color = 'orange'
            else:
                mode_text += "Approaching"
                mode_color = 'blue'
        elif status.is_const_current:
            mode_text += "Constant Current"
            mode_color = 'green'
        elif status.is_scanning:
            mode_text += "Scanning"
            mode_color = 'purple'
        else:
            mode_text += "Idle"
            mode_color = 'gray'
        self.status_mode.config(text=mode_text, foreground=mode_color)
        
        # Current reading
        current_amps = stm_control.STM_Status.adc_to_amp(status.adc)
        current_nanoamps = current_amps * 1e9
        adc_text = f"Current: {current_nanoamps:.2f} nA (ADC: {status.adc})"
        self.status_current.config(text=adc_text)
        
        # Position with distance calculation
        distance_um = stm_control.STM_Status.steps_to_distance(status.steps)
        if abs(distance_um) >= 1000:
            # Show in mm if >= 1mm
            distance_mm = distance_um / 1000
            position_text = f"Steps: {status.steps} ({distance_mm:.3f} mm)"
        else:
            # Show in μm
            position_text = f"Steps: {status.steps} ({distance_um:.2f} μm)"
        self.status_position.config(text=position_text)
        
        # Status message
        if status.is_approaching:
            direction = "Forward" if status.approach_direction == 1 else "Backward"
            if status.approach_recovery:
                self.status_message.config(text=f"Recovering from overshoot - {direction}", foreground='orange')
                # Update approach control status
                if hasattr(self, 'approach_control'):
                    self.approach_control.approach_status_label.config(
                        text="⚠️ RECOVERY MODE - Retracting tip", 
                        foreground='orange'
                    )
            else:
                self.status_message.config(text=f"Approaching tip - {direction}", foreground='blue')
                # Update approach control status
                if hasattr(self, 'approach_control'):
                    self.approach_control.approach_status_label.config(
                        text=f"▶️ Approaching ({direction})", 
                        foreground='blue'
                    )
        elif status.is_const_current:
            self.status_message.config(text="Maintaining constant current", foreground='green')
            # Clear approach status
            if hasattr(self, 'approach_control'):
                self.approach_control.approach_status_label.config(text="")
        elif status.is_scanning:
            self.status_message.config(text="Scanning in progress...", foreground='purple')
            # Clear approach status
            if hasattr(self, 'approach_control'):
                self.approach_control.approach_status_label.config(text="")
        elif self.stm.busy:
            self.status_message.config(text="Busy...", foreground='orange')
        else:
            self.status_message.config(text="Ready", foreground='black')
            # Clear approach status
            if hasattr(self, 'approach_control'):
                self.approach_control.approach_status_label.config(text="")
    
    def _update_images(self):
        if np.any(self.stm.scan_adc):
            self.scan_adc_frame.update_image(self.stm.scan_adc)
            self.scan_dacz_frame.update_image(self.stm.scan_dacz)
        self.after_id_images = self.after(100, self._update_images)

    def on_closing(self):
        if self.after_id:
            self.after_cancel(self.after_id)
        if self.after_id_images:
            self.after_cancel(self.after_id_images)
        self.destroy()

    def _plot_iv_curve(self, *args):
        try:
            int_args = [int(arg) for arg in args]
            iv_values = self.stm.measure_iv_curve(*int_args)
            if not iv_values: return
            num_points = iv_values[0]
            data = iv_values[1:]
            iv_pairs = list(zip(data[::2], data[1::2]))
            iv_pairs.sort(key=lambda x: x[0])
            sorted_bias_dac = [p[0] for p in iv_pairs]
            sorted_current_adc = [p[1] for p in iv_pairs]

            bias_volts = [stm_control.STM_Status.dac_to_bias_volts(d) for d in sorted_bias_dac]
            current_amps = [stm_control.STM_Status.adc_to_amp(a) for a in sorted_current_adc]
            self.iv_curve_frame.update_plot(bias_volts, current_amps)
            self.last_iv_data = list(zip(bias_volts, current_amps))

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

    def _show_pid_debug(self):
        """Display PID debug information in a popup window."""
        debug_info = self.stm.get_pid_debug()
        
        debug_window = tk.Toplevel(self)
        debug_window.title("PID Debug Information")
        debug_window.geometry("400x200")
        
        # Create text widget with scrollbar
        text_frame = ttk.Frame(debug_window)
        text_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        text_widget = tk.Text(text_frame, wrap=tk.WORD, height=8)
        scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=text_widget.yview)
        text_widget.configure(yscrollcommand=scrollbar.set)
        
        text_widget.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Insert debug information
        text_widget.insert(tk.END, f"PID Debug Information:\n\n{debug_info}\n\n")
        text_widget.insert(tk.END, "Tuning Tips:\n")
        text_widget.insert(tk.END, "• Increase Kp for faster response\n")
        text_widget.insert(tk.END, "• Increase Ki to eliminate steady-state error\n")
        text_widget.insert(tk.END, "• Increase Kd to reduce oscillations\n")
        text_widget.insert(tk.END, "• Start with small values and increase gradually\n")
        
        text_widget.config(state=tk.DISABLED)
        
        # Add refresh button
        refresh_button = ttk.Button(debug_window, text="Refresh", 
                                  command=lambda: self._refresh_pid_debug(text_widget))
        refresh_button.pack(pady=5)

    def _refresh_pid_debug(self, text_widget):
        """Refresh PID debug information."""
        debug_info = self.stm.get_pid_debug()
        text_widget.config(state=tk.NORMAL)
        text_widget.delete(1.0, tk.END)
        text_widget.insert(tk.END, f"PID Debug Information:\n\n{debug_info}\n\n")
        text_widget.insert(tk.END, "Tuning Tips:\n")
        text_widget.insert(tk.END, "• Increase Kp for faster response\n")
        text_widget.insert(tk.END, "• Increase Ki to eliminate steady-state error\n")
        text_widget.insert(tk.END, "• Increase Kd to reduce oscillations\n")
        text_widget.insert(tk.END, "• Start with small values and increase gradually\n")
        text_widget.config(state=tk.DISABLED)

    def _show_pid_tuning_guide(self):
        """Display comprehensive PID tuning guide."""
        guide_win = tk.Toplevel(self)
        guide_win.title("PID Tuning Guide for STM")
        guide_win.geometry("700x800")
        
        # Create scrollable frame
        outer_frame = ttk.Frame(guide_win)
        outer_frame.pack(fill='both', expand=True)
        outer_frame.grid_rowconfigure(0, weight=1)
        outer_frame.grid_columnconfigure(0, weight=1)
        
        canvas = tk.Canvas(outer_frame)
        canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar = ttk.Scrollbar(outer_frame, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky='ns')
        canvas.configure(yscrollcommand=scrollbar.set)
        
        content_frame = ttk.Frame(canvas, padding=20)
        canvas_frame_id = canvas.create_window((0, 0), window=content_frame, anchor="nw")
        
        def on_frame_cfg(e): canvas.configure(scrollregion=canvas.bbox("all"))
        def on_canvas_cfg(e): canvas.itemconfig(canvas_frame_id, width=e.width)
        content_frame.bind("<Configure>", on_frame_cfg)
        canvas.bind("<Configure>", on_canvas_cfg)
        
        # Title using ttk
        title = ttk.Label(content_frame, text="PID Controller Tuning Guide", 
                        font=('Arial', 16, 'bold'))
        title.pack(pady=(0, 20))
        
        # PID Equation using ttk
        eq_frame = ttk.Frame(content_frame, relief=tk.RAISED, borderwidth=2)
        eq_frame.pack(fill='x', pady=10)
        ttk.Label(eq_frame, text="📐 PID Control Equation", font=('Arial', 12, 'bold')).pack(pady=5)
        ttk.Label(eq_frame, text="Output(t) = Kp × e(t) + Ki × ∫e(τ)dτ + Kd × de(t)/dt", 
                font=('Courier', 11)).pack(pady=5)
        ttk.Label(eq_frame, text="where e(t) = Setpoint - Measured Value", 
                font=('Arial', 9, 'italic')).pack(pady=(0, 5))
        
        # Parameter explanations using ttk for theme support
        params = [
            ("Kp - Proportional Gain",
             "• Immediate response to current error\n"
             "• Effect: Output = Kp × error\n"
             "• ↑ Kp → Faster response, but may overshoot\n"
             "• ↓ Kp → Slower, more stable response\n"
             "• Too high → Oscillations and instability\n"
             "• Typical: 0.0001 - 0.01"),
            
            ("Ki - Integral Gain",
             "• Eliminates steady-state error over time\n"
             "• Effect: Output += Ki × ∫error dt\n"
             "• Accumulates past errors\n"
             "• ↑ Ki → Faster error elimination\n"
             "• ↓ Ki → Slower convergence to setpoint\n"
             "• Too high → Overshoot, slow oscillations\n"
             "• Typical: 0.00001 - 0.001"),
            
            ("Kd - Derivative Gain",
             "• Predicts future error based on rate of change\n"
             "• Effect: Output += Kd × (Δerror/Δt)\n"
             "• Dampens oscillations\n"
             "• ↑ Kd → More damping, reduced overshoot\n"
             "• ↓ Kd → Less damping, faster response\n"
             "• Too high → Amplifies noise, sluggish\n"
             "• Typical: 0.0 - 0.0001 (often 0 for STM)")
        ]
        
        for param_name, description in params:
            param_frame = ttk.Frame(content_frame, relief=tk.SOLID, borderwidth=1)
            param_frame.pack(fill='x', pady=8)
            
            header = ttk.Label(param_frame, text=param_name, font=('Arial', 11, 'bold'), 
                            anchor='w', padding=(10, 5))
            header.pack(fill='x')
            
            desc = ttk.Label(param_frame, text=description, font=('Arial', 10), 
                          anchor='w', justify='left', padding=(15, 10))
            desc.pack(fill='x')
        
        # Tuning methods using ttk
        tuning_frame = ttk.Frame(content_frame, relief=tk.RAISED, borderwidth=2)
        tuning_frame.pack(fill='x', pady=15)
        
        ttk.Label(tuning_frame, text="🔧 Tuning Methods", font=('Arial', 12, 'bold')).pack(pady=5)
        
        methods_text = """
Method 1: Manual Tuning (Recommended for STM)
1. Set Ki = 0, Kd = 0
2. Increase Kp until system responds quickly but oscillates slightly
3. Add small Ki (0.0001) to eliminate steady-state error
4. Add small Kd (0.00001) if oscillations persist
5. Fine-tune all three parameters

Method 2: Ziegler-Nichols
1. Set Ki = 0, Kd = 0
2. Increase Kp until sustained oscillation (Kp_critical)
3. Measure oscillation period T_critical (seconds)
4. Calculate:
   Kp = 0.6 × Kp_critical
   Ki = 2 × Kp / T_critical  
   Kd = Kp × T_critical / 8

Quick Start Values for STM:
   Kp = 0.0001
   Ki = 0.0001
   Kd = 0.0
        """
        
        ttk.Label(tuning_frame, text=methods_text, font=('Arial', 9), 
                anchor='w', justify='left', padding=(15, 10)).pack(fill='x')
        
        # Troubleshooting using ttk
        trouble_frame = ttk.Frame(content_frame, relief=tk.RAISED, borderwidth=2)
        trouble_frame.pack(fill='x', pady=10)
        
        ttk.Label(trouble_frame, text="⚠️ Troubleshooting", font=('Arial', 12, 'bold')).pack(pady=5)
        
        trouble_text = """
Problem: System oscillates continuously
→ Reduce Kp, reduce Ki, or increase Kd

Problem: Slow response, doesn't reach setpoint
→ Increase Kp, increase Ki

Problem: Overshoots then settles
→ Reduce Kp, increase Kd

Problem: Steady-state error (doesn't reach target)
→ Increase Ki

Problem: Noisy, erratic behavior
→ Reduce Kd, reduce Kp, increase filtering
        """
        
        ttk.Label(trouble_frame, text=trouble_text, font=('Arial', 9), 
                anchor='w', justify='left', padding=(15, 10)).pack(fill='x')
        
        # Close button
        ttk.Button(content_frame, text="Close", command=guide_win.destroy).pack(pady=20)
    
    def _update_z_range_display(self, event=None):
        """Update the distance display when Z range value changes."""
        try:
            z_range = int(self.z_range_var.get())
            # Convert DAC value to piezo displacement in nanometers
            displacement_nm = stm_control.STM_Status.dac_to_piezo_displacement(z_range, axis='z')
            if abs(displacement_nm) >= 1000:
                displacement_um = displacement_nm / 1000
                self.z_range_distance_label.config(text=f"≈ {displacement_um:.2f} μm")
            else:
                self.z_range_distance_label.config(text=f"≈ {displacement_nm:.1f} nm")
        except (ValueError, AttributeError):
            self.z_range_distance_label.config(text="≈ --- nm")
    
    def _set_fine_step_size(self):
        """Set the fine approach piezo Z step size."""
        try:
            step_size = int(self.fine_step_var.get())
            if self.stm.is_opened:
                self.stm.set_approach_fine_step_size(step_size)
                print(f"Fine Z step size set to: {step_size}")
        except ValueError:
            print("Invalid fine step size value")
    
    def _set_z_range(self):
        """Set the approach Z sweep search range."""
        try:
            z_range = int(self.z_range_var.get())
            if self.stm.is_opened:
                self.stm.set_approach_z_range(z_range)
                print(f"Z search range set to: {z_range}")
        except ValueError:
            print("Invalid Z range value")
    
    def _show_approach_debug(self):
        """Display approach debug information in a popup window."""
        debug_info = self.stm.get_approach_debug()
        
        debug_window = tk.Toplevel(self)
        debug_window.title("Approach Debug Information")
        debug_window.geometry("500x300")
        
        # Create text widget with scrollbar
        text_frame = ttk.Frame(debug_window)
        text_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        text_widget = tk.Text(text_frame, wrap=tk.WORD, height=12)
        scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=text_widget.yview)
        text_widget.configure(yscrollcommand=scrollbar.set)
        
        text_widget.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Insert debug information
        text_widget.insert(tk.END, f"Approach Debug Information:\n\n{debug_info}\n\n")
        text_widget.insert(tk.END, "Smart Approach Features:\n")
        text_widget.insert(tk.END, "• Overshoot Detection: Detects when tip crashes into surface\n")
        text_widget.insert(tk.END, "• Recovery Mode: Automatically retracts tip when overshoot detected\n")
        text_widget.insert(tk.END, "• Fine Approach: Uses smaller steps after recovery\n")
        text_widget.insert(tk.END, "• Adaptive Range: Expands search range if target not found\n\n")
        text_widget.insert(tk.END, "Troubleshooting:\n")
        text_widget.insert(tk.END, "• If overshoot occurs frequently, reduce approach speed\n")
        text_widget.insert(tk.END, "• If recovery fails, check Z-piezo range and tip condition\n")
        text_widget.insert(tk.END, "• Monitor ADC values to ensure proper current detection\n")
        
        text_widget.config(state=tk.DISABLED)
        
        # Add control buttons
        button_frame = ttk.Frame(debug_window)
        button_frame.pack(pady=5)
        
        refresh_button = ttk.Button(button_frame, text="Refresh", 
                                  command=lambda: self._refresh_approach_debug(text_widget))
        refresh_button.pack(side='left', padx=5)
        
        recovery_button = ttk.Button(button_frame, text="Trigger Recovery (Test)", 
                                   command=self.stm.trigger_approach_recovery)
        recovery_button.pack(side='left', padx=5)

    def _refresh_approach_debug(self, text_widget):
        """Refresh approach debug information."""
        debug_info = self.stm.get_approach_debug()
        text_widget.config(state=tk.NORMAL)
        text_widget.delete(1.0, tk.END)
        text_widget.insert(tk.END, f"Approach Debug Information:\n\n{debug_info}\n\n")
        text_widget.insert(tk.END, "Smart Approach Features:\n")
        text_widget.insert(tk.END, "• Overshoot Detection: Detects when tip crashes into surface\n")
        text_widget.insert(tk.END, "• Recovery Mode: Automatically retracts tip when overshoot detected\n")
        text_widget.insert(tk.END, "• Fine Approach: Uses smaller steps after recovery\n")
        text_widget.insert(tk.END, "• Adaptive Range: Expands search range if target not found\n\n")
        text_widget.insert(tk.END, "Troubleshooting:\n")
        text_widget.insert(tk.END, "• If overshoot occurs frequently, reduce approach speed\n")
        text_widget.insert(tk.END, "• If recovery fails, check Z-piezo range and tip condition\n")
        text_widget.insert(tk.END, "• Monitor ADC values to ensure proper current detection\n")
        text_widget.config(state=tk.DISABLED)

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app._quit)
    app.mainloop()