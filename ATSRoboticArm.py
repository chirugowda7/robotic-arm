import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import json
import pygame
from datetime import datetime
from collections import deque
import csv

class IndustrialRoboticArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("Industrial Robotic Arm Controller - Speed 100")
        self.root.geometry("1600x1000")
        
        # Serial connection
        self.serial_port = None
        self.connected = False
        self.serial_lock = threading.Lock()
        
        # Joystick
        self.joystick = None
        self.stick_connected = False
        self.joystick_thread = None
        self.joystick_running = False
        self.joystick_deadzone = 0.15
        
        # Recording
        self.recording = False
        self.recorded_data = []
        self.recording_thread = None
        
        # Playback
        self.playing = False
        self.playback_thread = None
        
        # Current angles (Sync with Home position)
        self.current_angles = [90, 90, 90, 90, 90, 90]
        self.target_angles = [90, 90, 90, 90, 90, 90]
        
        # Movement control
        self.movement_mode = "smooth"
        self.movement_speed = 100.0
        self.movement_state = "IDLE"
        
        # Cell positions
        self.cell_positions = {
            0: [0, 100, 175, 115, 145, 90],
            1: [29, 64, 180, 121, 145, 90],
            2: [85, 63, 175, 130, 145, 90],
            3: [46, 61, 175, 130, 145, 90],
            4: [85, 94, 167, 117, 145, 90],
            5: [85, 94, 167, 117, 145, 90],
            6: [85, 94, 167, 117, 145, 90],
            7: [85, 94, 167, 117, 145, 90],
            8: [85, 94, 167, 117, 145, 90],
            9: [85, 94, 167, 117, 145, 90]
        }
        
        # System status
        self.system_uptime = 0
        self.movement_count = 0
        self.log_size = 0
        self.emergency_stop_active = False
        
        # Joystick control
        self.joystick_control_enabled = False
        self.last_joystick_update = 0
        self.joystick_update_interval = 0.1
        self.joystick_sensitivity = 1.0
        
        # 
        #  limits
        self.servo_limits = [
            (0, 180),
            (30, 150),
            (60, 180),
            (30, 150),
            (0, 180),
            (0, 180)
        ]
        
        # Movement smoothing
        self.smoothing_buffers = [deque(maxlen=5) for _ in range(6)]
        
        # Wrist increment
        self.wrist_increment = 5
        
        # Debug mode
        self.debug_mode = True
        
        # Setup UI
        self.setup_ui()
        
        # Initialize joystick
        self.init_joystick()
        
        # Start periodic status updates
        self.update_system_status()
        
    def setup_ui(self):
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Control Tab
        self.control_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.control_frame, text='Manual Control')
        self.setup_control_tab()
        
        # Cell Operations Tab
        self.cell_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.cell_frame, text='Cell Operations')
        self.setup_cell_tab()
        
        # Joystick Tab
        self.joystick_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.joystick_frame, text='Joystick Control')
        self.setup_joystick_tab()
        
        # Recording Tab
        self.recording_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.recording_frame, text='Recording & Playback')
        self.setup_recording_tab()
        
        # System Status Tab
        self.status_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.status_frame, text='System Status')
        self.setup_status_tab()
        
        # Status Bar
        self.setup_status_bar()
        
    def setup_control_tab(self):
        # Connection frame
        conn_frame = ttk.LabelFrame(self.control_frame, text="Connection", padding=10)
        conn_frame.grid(row=0, column=0, columnspan=3, sticky='ew', padx=5, pady=5)
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0)
        self.port_var = tk.StringVar(value="COM9")
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)
        
        self.refresh_ports_btn = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_ports_btn.grid(row=0, column=2, padx=5)
        
        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2)
        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, 
                                       values=['9600', '19200', '38400', '57600', '115200'], width=10)
        self.baud_combo.grid(row=0, column=3, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5)
        
        self.emergency_btn = ttk.Button(conn_frame, text="EMERGENCY STOP", 
                                        command=self.emergency_stop, style='Emergency.TButton')
        self.emergency_btn.grid(row=0, column=5, padx=5)
        
        self.reset_estop_btn = ttk.Button(conn_frame, text="Reset E-Stop", 
                                         command=self.reset_emergency_stop, style='Reset.TButton')
        self.reset_estop_btn.grid(row=0, column=6, padx=5)
        
        # Debug button
        self.debug_btn = ttk.Button(conn_frame, text="Debug ON", 
                                   command=self.toggle_debug)
        self.debug_btn.grid(row=0, column=7, padx=5)
        
        # Servo sliders
        self.slider_frame = ttk.LabelFrame(self.control_frame, text="Servo Control", padding=10)
        self.slider_frame.grid(row=1, column=0, columnspan=3, sticky='nsew', padx=5, pady=5)
        
        self.sliders = []
        self.slider_vars = []
        self.servo_labels = []
        self.limit_labels = []
        
        servo_names = ['Base', 'Shoulder', 'Elbow', 'Wrist Pitch', 'Wrist Roll', 'Wrist Rotate']
        
        for i in range(6):
            frame = ttk.Frame(self.slider_frame)
            frame.grid(row=i, column=0, sticky='ew', pady=2)
            
            ttk.Label(frame, text=f"{servo_names[i]} ({i+1}):", width=15).grid(row=0, column=0)
            
            var = tk.DoubleVar(value=self.current_angles[i])
            self.slider_vars.append(var)
            
            slider = ttk.Scale(frame, from_=self.servo_limits[i][0], to=self.servo_limits[i][1], 
                              variable=var, orient='horizontal', length=300,
                              command=lambda val, idx=i: self.on_slider_change(idx, val))
            slider.grid(row=0, column=1, padx=10)
            
            label = ttk.Label(frame, text=f"{self.current_angles[i]:.1f}°", width=10)
            label.grid(row=0, column=2)
            self.servo_labels.append(label)
            
            limit_label = ttk.Label(frame, text=f"({self.servo_limits[i][0]}°-{self.servo_limits[i][1]}°)", 
                                   foreground='gray', font=('Arial', 8))
            limit_label.grid(row=0, column=3, padx=5)
            self.limit_labels.append(limit_label)
            
            ttk.Button(frame, text="Set", 
                      command=lambda idx=i: self.set_single_servo(idx)).grid(row=0, column=4, padx=5)
            
            self.sliders.append(slider)
        
        # Preset positions frame
        preset_frame = ttk.LabelFrame(self.control_frame, text="Preset Positions", padding=10)
        preset_frame.grid(row=2, column=0, sticky='nsew', padx=5, pady=5)
        
        presets = {
            "Home": [0.0, 114.0, 90.0, 112.0, 147.0, 90.0],
            "Safe": [45, 90, 173, 119, 75, 90],
            "Maintenance": [0, 90, 175, 115, 145, 90],
            "Pick Position": [0, 90, 175, 115, 145, 90],
            "Place Position": [0, 90, 175, 115, 145, 90],
            "Transport": [0, 90, 175, 115, 145, 90]
        }
        
        row, col = 0, 0
        for name, angles in presets.items():
            btn = ttk.Button(preset_frame, text=name,
                            command=lambda a=angles, n=name: self.move_to_preset(a, n))
            btn.grid(row=row, column=col, padx=5, pady=5, sticky='ew')
            col += 1
            if col > 1:
                col = 0
                row += 1
        
        # Gripper control
        gripper_frame = ttk.LabelFrame(self.control_frame, text="Relay / Gripper Control", padding=10)
        gripper_frame.grid(row=2, column=1, sticky='nsew', padx=5, pady=5)
        
        self.gripper_state = tk.StringVar(value="OFF")
        ttk.Label(gripper_frame, text="Relay State:").pack()
        ttk.Label(gripper_frame, textvariable=self.gripper_state, 
                 font=('Arial', 12, 'bold'), foreground='blue').pack()
        
        btn_frame = ttk.Frame(gripper_frame)
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="Relay OFF (Open)", 
                  command=lambda: self.send_gripper_command("OFF")).pack(side='left', padx=2, fill='x', expand=True)
        ttk.Button(btn_frame, text="Relay ON (Close)", 
                  command=lambda: self.send_gripper_command("ON")).pack(side='left', padx=2, fill='x', expand=True)
        ttk.Button(btn_frame, text="Toggle", 
                  command=self.toggle_gripper).pack(side='left', padx=2, fill='x', expand=True)
        
        # Position control frame
        pos_frame = ttk.LabelFrame(self.control_frame, text="Position Control", padding=10)
        pos_frame.grid(row=2, column=2, sticky='nsew', padx=5, pady=5)
        
        ttk.Label(pos_frame, text="Get Position:").pack(anchor='w')
        ttk.Button(pos_frame, text="Read Current", 
                  command=self.get_current_position).pack(fill='x', pady=2)
        ttk.Button(pos_frame, text="Send Position", 
                  command=self.send_current_position).pack(fill='x', pady=2)
        ttk.Button(pos_frame, text="Save Position", 
                  command=self.save_position).pack(fill='x', pady=2)
        ttk.Button(pos_frame, text="Test Connection", 
                  command=self.test_connection).pack(fill='x', pady=2)
        
        # Configure grid weights
        self.control_frame.columnconfigure(0, weight=1)
        self.control_frame.columnconfigure(1, weight=1)
        self.control_frame.columnconfigure(2, weight=1)
        
    def setup_cell_tab(self):
        # Main container with two columns
        self.cell_main_frame = ttk.Frame(self.cell_frame)
        self.cell_main_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # LEFT: Cell Grid Selection
        grid_container = ttk.LabelFrame(self.cell_main_frame, text="Select Cell (3x3 Grid)", padding=10)
        grid_container.pack(side='left', fill='both', expand=False, padx=5)
        
        self.cell_buttons = []
        self.selected_cell_idx = tk.IntVar(value=1)
        
        for row in range(3):
            row_frame = ttk.Frame(grid_container)
            row_frame.pack(pady=5)
            for col in range(3):
                cell_num = row * 3 + col + 1
                # Use Radiobutton styled as a button for selection
                btn = ttk.Radiobutton(row_frame, text=f"Cell {cell_num}", 
                                     variable=self.selected_cell_idx, value=cell_num,
                                     style='Toolbutton', width=10,
                                     command=self.on_cell_selected)
                btn.pack(side='left', padx=5)
                self.cell_buttons.append(btn)
        
        # Action buttons for grid
        grid_ops = ttk.Frame(grid_container)
        grid_ops.pack(fill='x', pady=20)
        ttk.Button(grid_ops, text="MOVE TO SELECTED", 
                  command=lambda: self.move_to_cell(self.selected_cell_idx.get())).pack(fill='x', pady=5)
        
        # RIGHT: Cell Editor
        self.editor_frame = ttk.LabelFrame(self.cell_main_frame, text="Cell Editor", padding=15)
        self.editor_frame.pack(side='left', fill='both', expand=True, padx=5)
        
        self.cell_title_label = ttk.Label(self.editor_frame, text="Editing Cell 1", font=('Arial', 12, 'bold'))
        self.cell_title_label.pack(pady=(0, 15))
        
        # Angle input fields
        inputs_frame = ttk.Frame(self.editor_frame)
        inputs_frame.pack(fill='x')
        
        self.cell_edit_vars = []
        servo_names = ['Base (S1)', 'Shoulder (S2)', 'Elbow (S3)', 'Wrist P (S4)', 'Wrist R (S5)', 'Rotate (S6)']
        
        for i in range(6):
            f = ttk.Frame(inputs_frame)
            f.pack(fill='x', pady=2)
            ttk.Label(f, text=servo_names[i], width=15).pack(side='left')
            
            var = tk.StringVar()
            self.cell_edit_vars.append(var)
            
            ent = ttk.Entry(f, textvariable=var, width=10)
            ent.pack(side='left', padx=5)
            
            limits = self.servo_limits[i]
            ttk.Label(f, text=f"({limits[0]} - {limits[1]})", foreground='gray').pack(side='left')
            
        # Editor controls
        edit_ops = ttk.Frame(self.editor_frame)
        edit_ops.pack(fill='x', pady=20)
        
        ttk.Button(edit_ops, text="TEACH (Copy Current Arm Pos)", 
                  command=self.teach_selected_cell).pack(fill='x', pady=2)
        
        btn_row = ttk.Frame(edit_ops)
        btn_row.pack(fill='x', pady=5)
        ttk.Button(btn_row, text="UPDATE / SAVE CELL", 
                  command=self.update_selected_cell_data).pack(side='left', fill='x', expand=True, padx=2)
        ttk.Button(btn_row, text="CLEAR / DELETE", 
                  command=self.delete_selected_cell_data).pack(side='left', fill='x', expand=True, padx=2)
        
        # File operations
        file_ops = ttk.LabelFrame(self.editor_frame, text="Library Management", padding=10)
        file_ops.pack(fill='x', side='bottom', pady=10)
        ttk.Button(file_ops, text="Load Library", command=self.load_cell_positions).pack(side='left', expand=True, fill='x', padx=2)
        ttk.Button(file_ops, text="Save Library", command=self.save_cell_positions).pack(side='left', expand=True, fill='x', padx=2)
        
        # Initialize display
        self.on_cell_selected()
        
    def setup_joystick_tab(self):
        # Joystick status
        status_frame = ttk.LabelFrame(self.joystick_frame, text="Joystick Status", padding=10)
        status_frame.pack(fill='x', padx=10, pady=10)
        
        self.joystick_status_label = ttk.Label(status_frame, text="Not connected", foreground='red')
        self.joystick_status_label.pack()
        
        self.joystick_name_label = ttk.Label(status_frame, text="")
        self.joystick_name_label.pack()
        
        # Joystick control toggle
        control_frame = ttk.Frame(status_frame)
        control_frame.pack(pady=10)
        
        self.enable_joystick_btn = ttk.Button(control_frame, text="Enable Joystick Control", 
                                             command=self.toggle_joystick_control)
        self.enable_joystick_btn.pack(side='left', padx=5)
        
        self.joystick_active_label = ttk.Label(control_frame, text="DISABLED", 
                                              foreground='red', font=('Arial', 10, 'bold'))
        self.joystick_active_label.pack(side='left', padx=10)
        
        # Sensitivity control
        sens_frame = ttk.Frame(status_frame)
        sens_frame.pack(pady=5)
        
        ttk.Label(sens_frame, text="Sensitivity:").pack(side='left')
        self.sens_var = tk.DoubleVar(value=1.0)
        sens_scale = ttk.Scale(sens_frame, from_=0.1, to=3.0, variable=self.sens_var, 
                              orient='horizontal', length=150)
        sens_scale.pack(side='left', padx=10)
        self.sens_label = ttk.Label(sens_frame, text="1.0")
        self.sens_label.pack(side='left')
        sens_scale.config(command=lambda v: self.sens_label.config(text=f"{self.sens_var.get():.1f}"))
        
        # Deadzone control
        deadzone_frame = ttk.Frame(status_frame)
        deadzone_frame.pack(pady=5)
        
        ttk.Label(deadzone_frame, text="Deadzone:").pack(side='left')
        self.deadzone_var = tk.DoubleVar(value=0.15)
        deadzone_scale = ttk.Scale(deadzone_frame, from_=0.05, to=0.5, variable=self.deadzone_var, 
                                  orient='horizontal', length=150)
        deadzone_scale.pack(side='left', padx=10)
        self.deadzone_label = ttk.Label(deadzone_frame, text="0.15")
        self.deadzone_label.pack(side='left')
        deadzone_scale.config(command=lambda v: self.deadzone_label.config(text=f"{self.deadzone_var.get():.2f}"))
        
        # Wrist increment control
        increment_frame = ttk.Frame(status_frame)
        increment_frame.pack(pady=5)
        
        ttk.Label(increment_frame, text="Wrist Increment:").pack(side='left')
        self.increment_var = tk.IntVar(value=5)
        increment_scale = ttk.Scale(increment_frame, from_=1, to=20, variable=self.increment_var,
                                   orient='horizontal', length=150)
        increment_scale.pack(side='left', padx=10)
        self.increment_label = ttk.Label(increment_frame, text="5°")
        self.increment_label.pack(side='left')
        increment_scale.config(command=lambda v: self.update_increment_display())
        
        # Axis visualization
        axis_frame = ttk.LabelFrame(self.joystick_frame, text="Axis Values (0-3)", padding=10)
        axis_frame.pack(fill='x', padx=10, pady=10)
        
        self.axis_labels = []
        for i in range(4):
            frame = ttk.Frame(axis_frame)
            frame.pack(fill='x', pady=2)
            
            ttk.Label(frame, text=f"Axis {i}:", width=10).pack(side='left')
            
            canvas = tk.Canvas(frame, width=200, height=20, bg='lightgray', highlightthickness=0)
            canvas.pack(side='left', padx=10)
            
            canvas.create_rectangle(0, 5, 200, 15, fill='gray', outline='')
            bar = canvas.create_rectangle(100, 5, 100, 15, fill='blue', outline='')
            
            label = ttk.Label(frame, text="0.000", width=10)
            label.pack(side='left')
            
            self.axis_labels.append((canvas, bar, label))
        
        # Button visualization
        button_frame = ttk.LabelFrame(self.joystick_frame, text="Button States (B0, B1, B2, B3)", padding=10)
        button_frame.pack(fill='x', padx=10, pady=10)
        
        self.button_canvases = []
        self.button_lights = []
        
        # Create container inside button_frame
        button_container = ttk.Frame(button_frame)
        button_container.pack()
        
        # Only create buttons for B0, B1, B2, B3
        button_indices = [0, 1, 2, 3]
        button_names = ["B0: Home", "B1: Wrist Roll +", "B2: Wrist Roll -", "B3: Relay Toggle"]
        
        for i, btn_idx in enumerate(button_indices):
            frame = ttk.Frame(button_container)
            frame.pack(side='left', padx=10, pady=5)
            
            canvas = tk.Canvas(frame, width=30, height=30, bg='lightgray', highlightthickness=1)
            canvas.pack()
            light = canvas.create_oval(5, 5, 25, 25, fill='red', outline='black')
            
            ttk.Label(frame, text=button_names[i]).pack()
            
            self.button_canvases.append(canvas)
            self.button_lights.append(light)
        
        # Control mapping info
        info_frame = ttk.LabelFrame(self.joystick_frame, text="Joystick Control Mapping", padding=10)
        info_frame.pack(fill='x', padx=10, pady=10)
        
        mapping_text = """JOYSTICK CONTROL MAPPING:

ANALOG STICKS:
Left Stick X (Axis 0) → Base Rotation
Left Stick Y (Axis 1) → Shoulder (INVERTED: forward=down, backward=up)
Right Stick X (Axis 2) → Elbow
Right Stick Y (Axis 3) → Wrist Pitch

BUTTON CONTROLS:
B0 → Go to Home Position
B1 → Wrist Roll INCREASE
B2 → Wrist Roll DECREASE
B3 → Toggle Relay (ON/OFF)

SETTINGS:
• Speed: 100 (Constant)
• Wrist Step: 1-20°
• Sensitivity: 0.1-3.0
• Deadzone: 0.05-0.50"""
        
        ttk.Label(info_frame, text=mapping_text, justify='left').pack(anchor='w')
        
    def setup_recording_tab(self):
        # Recording controls
        record_frame = ttk.LabelFrame(self.recording_frame, text="Recording", padding=10)
        record_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Button(record_frame, text="Start Recording", 
                  command=self.start_recording).pack(side='left', padx=5)
        ttk.Button(record_frame, text="Stop Recording", 
                  command=self.stop_recording).pack(side='left', padx=5)
        ttk.Button(record_frame, text="Save Recording", 
                  command=self.save_recording).pack(side='left', padx=5)
        ttk.Button(record_frame, text="Load Recording", 
                  command=self.load_recording).pack(side='left', padx=5)
        
        self.recording_status = ttk.Label(record_frame, text="Not recording", foreground='red')
        self.recording_status.pack(side='left', padx=20)
        
        # Data logging controls
        log_frame = ttk.LabelFrame(self.recording_frame, text="Data Logging", padding=10)
        log_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Button(log_frame, text="Start Logging", 
                  command=self.start_logging).pack(side='left', padx=5)
        ttk.Button(log_frame, text="Stop Logging", 
                  command=self.stop_logging).pack(side='left', padx=5)
        ttk.Button(log_frame, text="Get Log Size", 
                  command=self.get_log_size).pack(side='left', padx=5)
        ttk.Button(log_frame, text="Clear Log", 
                  command=self.clear_log).pack(side='left', padx=5)
        ttk.Button(log_frame, text="Export CSV", 
                  command=self.export_csv).pack(side='left', padx=5)
        
        # Playback controls
        playback_frame = ttk.LabelFrame(self.recording_frame, text="Playback", padding=10)
        playback_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Button(playback_frame, text="Start Playback", 
                  command=self.start_playback).pack(side='left', padx=5)
        ttk.Button(playback_frame, text="Stop Playback", 
                  command=self.stop_playback).pack(side='left', padx=5)
        
        # Recording list
        list_frame = ttk.LabelFrame(self.recording_frame, text="Recorded Waypoints", padding=10)
        list_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        columns = ('Time', 'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'Gripper')
        self.recording_tree = ttk.Treeview(list_frame, columns=columns, show='headings', height=15)
        
        for col in columns:
            self.recording_tree.heading(col, text=col)
            self.recording_tree.column(col, width=80)
        
        scrollbar = ttk.Scrollbar(list_frame, orient='vertical', 
                                 command=self.recording_tree.yview)
        self.recording_tree.configure(yscrollcommand=scrollbar.set)
        
        self.recording_tree.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
    def setup_status_tab(self):
        # System information
        sys_frame = ttk.LabelFrame(self.status_frame, text="System Information", padding=10)
        sys_frame.pack(fill='x', padx=10, pady=10)
        
        self.status_labels = {}
        status_items = [
            ("Uptime:", "uptime_label", "0s"),
            ("Movement Count:", "move_count_label", "0"),
            ("Log Size:", "log_size_label", "0"),
            ("System State:", "system_state_label", "INITIALIZING"),
            ("Emergency Stop:", "estop_label", "INACTIVE"),
            ("Connection:", "conn_label", "DISCONNECTED"),
            ("Speed Mode:", "speed_label", "100 (Constant)"),
            ("Control Mode:", "control_label", "SMOOTH"),
            ("Joystick:", "joystick_label", "DISCONNECTED")
        ]
        
        for i, (text, key, default) in enumerate(status_items):
            frame = ttk.Frame(sys_frame)
            frame.grid(row=i//2, column=i%2*2, sticky='w', padx=5, pady=2)
            
            ttk.Label(frame, text=text, width=15).pack(side='left')
            label = ttk.Label(frame, text=default, width=20, font=('Arial', 9, 'bold'))
            label.pack(side='left')
            self.status_labels[key] = label
        
        # Safety limits display
        safety_frame = ttk.LabelFrame(self.status_frame, text="Safety Limits", padding=10)
        safety_frame.pack(fill='x', padx=10, pady=10)
        
        columns = ('Servo', 'Name', 'Min Angle', 'Max Angle', 'Current')
        tree = ttk.Treeview(safety_frame, columns=columns, show='headings', height=7)
        
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, width=100)
        
        servo_names = ['Base', 'Shoulder', 'Elbow', 'Wrist Pitch', 'Wrist Roll', 'Wrist Rotate']
        
        for i in range(6):
            tree.insert('', 'end', values=(
                i+1,
                servo_names[i],
                f"{self.servo_limits[i][0]}°",
                f"{self.servo_limits[i][1]}°",
                f"{self.current_angles[i]:.1f}°"
            ))
        
        scrollbar = ttk.Scrollbar(safety_frame, orient='vertical', command=tree.yview)
        tree.configure(yscrollcommand=scrollbar.set)
        
        tree.pack(side='left', fill='both', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # System commands
        cmd_frame = ttk.LabelFrame(self.status_frame, text="System Commands", padding=10)
        cmd_frame.pack(fill='x', padx=10, pady=10)
        
        ttk.Button(cmd_frame, text="Refresh Status", 
                  command=self.refresh_system_status).pack(side='left', padx=5)
        ttk.Button(cmd_frame, text="Get System Status", 
                  command=self.get_system_status).pack(side='left', padx=5)
        ttk.Button(cmd_frame, text="Diagnostic Test", 
                  command=self.run_diagnostic).pack(side='left', padx=5)
        ttk.Button(cmd_frame, text="Calibrate Servos", 
                  command=self.calibrate_servos).pack(side='left', padx=5)
        
    def setup_status_bar(self):
        self.status_bar = ttk.Frame(self.root)
        self.status_bar.pack(side='bottom', fill='x')
        
        self.status_label = ttk.Label(self.status_bar, text="Industrial Robotic Arm Controller - Speed 100 | Disconnected", 
                                     relief='sunken')
        self.status_label.pack(side='left', fill='x', expand=True)
        
        self.connection_label = ttk.Label(self.status_bar, text="Port: None | Mode: SMOOTH", 
                                         relief='sunken', width=30)
        self.connection_label.pack(side='right')
        
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            if "COM9" in ports:
                self.port_combo.set("COM9")
            else:
                self.port_combo.set(ports[0])
        else:
            self.port_combo.set("")

    def toggle_connection(self):
        if not self.connected:
            try:
                port = self.port_var.get()
                if not port:
                    messagebox.showwarning("Connection", "Please select a COM port")
                    return
                
                baud = int(self.baud_var.get())
                
                print(f"DEBUG: Connecting to {port} at {baud} baud")
                
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=baud,
                    timeout=0.1, # Short timeout for better responsiveness
                    write_timeout=1
                )
                
                time.sleep(1.5) # Wait for ESP32 to reboot
                
                if self.serial_port.in_waiting > 0:
                    self.serial_port.read(self.serial_port.in_waiting)
                
                self.connected = True
                self.connect_btn.config(text="Disconnect")
                self.status_label.config(text=f"Connected to {port}")
                self.connection_label.config(text=f"Port: {port} | Mode: {self.movement_mode.upper()}")
                self.status_labels['conn_label'].config(text="CONNECTED", foreground='green')
                
                self.serial_thread = threading.Thread(target=self.serial_reader)
                self.serial_thread.daemon = True
                self.serial_thread.start()
                
                self.send_command("DEBUG_ON")
                self.set_movement_mode(self.movement_mode)
                self.get_current_position()
                
            except Exception as e:
                self.connected = False
                if self.serial_port:
                    try: self.serial_port.close()
                    except: pass
                self.serial_port = None
                messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
                print(f"ERROR: {e}")
        else:
            self.cleanup_connection()

    def cleanup_connection(self):
        self.connected = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
            self.serial_port = None
        
        self.root.after(0, self.update_ui_disconnected)

    def update_ui_disconnected(self):
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected")
        self.connection_label.config(text="Port: None | Mode: SMOOTH")
        self.status_labels['conn_label'].config(text="DISCONNECTED", foreground='red')

    def serial_reader(self):
        while self.connected:
            if not self.serial_port or not self.serial_port.is_open:
                break
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.process_serial_message(line)
            except (serial.SerialException, OSError) as e:
                print(f"Connection lost: {e}")
                self.root.after(0, lambda: self.handle_disconnection())
                break
            except Exception as e:
                print(f"Serial read error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)
        
        self.cleanup_connection()

    def handle_disconnection(self):
        if self.connected:
            messagebox.showerror("Connection Lost", "The robotic arm was disconnected.")
            self.cleanup_connection()
            
    def send_command(self, command):
        if self.connected and self.serial_port:
            try:
                with self.serial_lock:
                    if self.debug_mode:
                        print(f"SENDING: {command}")
                    self.serial_port.write((command + '\n').encode('utf-8'))
                    self.serial_port.flush()
            except Exception as e:
                print(f"Send error: {e}")
                self.connected = False
                self.connect_btn.config(text="Connect")
    
    def process_serial_message(self, message):
        """Process messages from the 5-servo ESP32 firmware."""
        message = message.strip()
        if not message:
            return
        if not all(32 <= ord(c) <= 126 for c in message):
            return

        if self.debug_mode:
            print(f"RECEIVED: {message}")

        if message.startswith("POSITION|"):
            parts = message.split("|")
            if len(parts) >= 6:
                try:
                    angles = [float(parts[i]) for i in range(1, 6)]
                    for i in range(5):
                        lo, hi = self.servo_limits[i]
                        angles[i] = max(lo, min(hi, angles[i]))
                        self.current_angles[i] = angles[i]
                        self.target_angles[i] = angles[i]
                        self.root.after(
                            0,
                            lambda idx=i, a=angles[i]:
                            self.update_servo_display(idx, a)
                        )
                    self.movement_state = "IDLE"
                except (ValueError, IndexError) as e:
                    print(f"Invalid POSITION message: {message} ({e})")

        elif message.startswith("CURRENT_POSITION|"):
            # Backward compatibility with an older 6-servo response.
            parts = message.split("|")
            if len(parts) >= 7:
                try:
                    angles = [float(parts[i + 1]) for i in range(6)]
                    for i in range(6):
                        lo, hi = self.servo_limits[i]
                        angles[i] = max(lo, min(hi, angles[i]))
                        self.current_angles[i] = angles[i]
                        self.target_angles[i] = angles[i]
                        self.root.after(
                            0,
                            lambda idx=i, a=angles[i]:
                            self.update_servo_display(idx, a)
                        )
                except (ValueError, IndexError) as e:
                    print(f"Invalid CURRENT_POSITION message: {message} ({e})")

        elif message.startswith("STATUS|"):
            parts = message.split("|")
            if len(parts) >= 2:
                state = parts[1].upper()
                if state == "ESTOP":
                    self.movement_state = "ESTOP"
                    self.emergency_stop_active = True
                elif state == "MOVING":
                    self.movement_state = "MOVING"
                else:
                    self.movement_state = "IDLE"
                self.root.after(0, self.refresh_system_status)

        elif message == "EMERGENCY_STOPPED":
            self.emergency_stop_active = True
            self.movement_state = "ESTOP"
            self.root.after(0, self.refresh_system_status)

        elif message == "ESTOP_RESET":
            self.emergency_stop_active = False
            self.movement_state = "IDLE"
            self.root.after(0, self.refresh_system_status)

        elif message.startswith("GRIPPER|"):
            parts = message.split("|")
            if len(parts) >= 2:
                display_state = "ON" if parts[1].upper() == "CLOSE" else "OFF"
                self.root.after(0, lambda s=display_state: self.gripper_state.set(s))

        elif message.startswith("SPEED|"):
            parts = message.split("|")
            if len(parts) >= 2:
                try:
                    self.movement_speed = float(parts[1])
                except ValueError:
                    pass

        elif message == "PONG":
            if self.debug_mode:
                print("ESP32 communication OK: PONG")

        elif message.startswith("ERROR|"):
            print(f"ESP32 ERROR: {message}")

        elif message.startswith(("DEBUG|", "MODE|")):
            if self.debug_mode:
                print(f"ESP32 INFO: {message}")
    
    # Recording methods
    def start_recording(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to arm")
            return
            
        self.recording = True
        self.recorded_data = []
        self.recording_status.config(text="Recording...", foreground='green')
        
        self.recording_thread = threading.Thread(target=self.record_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
    def stop_recording(self):
        self.recording = False
        self.recording_status.config(text="Recording stopped", foreground='red')
        
    def record_loop(self):
        start_time = time.time()
        while self.recording and self.connected:
            data = {
                'timestamp': time.time() - start_time,
                'angles': self.current_angles.copy(),
                'gripper': self.gripper_state.get()
            }
            self.recorded_data.append(data)
            time.sleep(0.05)
        
    def save_recording(self):
        if not self.recorded_data:
            messagebox.showwarning("No Data", "No recording data to save")
            return
            
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            initialfile=f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            with open(filename, 'w') as f:
                json.dump(self.recorded_data, f, indent=4)
            messagebox.showinfo("Saved", f"Recording saved to {filename}")
        
    def load_recording(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json")]
        )
        if filename:
            with open(filename, 'r') as f:
                self.recorded_data = json.load(f)
            self.update_recording_treeview()
            
    def update_recording_treeview(self):
        self.recording_tree.delete(*self.recording_tree.get_children())
        for i, data in enumerate(self.recorded_data):
            self.recording_tree.insert('', 'end', values=(
                f"{data['timestamp']:.1f}s",
                data['angles'][0],
                data['angles'][1],
                data['angles'][2],
                data['angles'][3],
                data['angles'][4],
                data['angles'][5],
                data['gripper']
            ))
        
    def start_playback(self):
        if not self.recorded_data:
            messagebox.showwarning("No Data", "No recording to play")
            return
            
        if not self.connected:
            messagebox.showerror("Error", "Not connected to arm")
            return
            
        self.playing = True
        self.playback_thread = threading.Thread(target=self.playback_loop)
        self.playback_thread.daemon = True
        self.playback_thread.start()
        
    def stop_playback(self):
        self.playing = False
        
    def playback_loop(self):
        if not self.recorded_data:
            return

        self.root.after(0, lambda: messagebox.showinfo("Playback", "Starting playback..."))

        for i, data in enumerate(self.recorded_data):
            if not self.playing or not self.connected:
                break

            try:
                angles = data.get("angles", [])
                if len(angles) < 6:
                    print("Playback error: waypoint has fewer than 6 angles.")
                    continue

                self.target_angles = [float(a) for a in angles[:6]]

                for servo_idx in range(6):
                    lo, hi = self.servo_limits[servo_idx]
                    self.target_angles[servo_idx] = max(
                        lo, min(hi, self.target_angles[servo_idx])
                    )

                physical = self.target_angles[:5]
                cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
                self.send_command(cmd)

                vals = self.target_angles.copy()
                self.root.after(0, lambda v=vals: self.update_all_servo_displays(v))

                recorded_gripper = str(data.get("gripper", "OFF")).upper()
                target_state = "OFF" if ("OPEN" in recorded_gripper or "OFF" in recorded_gripper) else "ON"
                if target_state != self.gripper_state.get():
                    self.send_gripper_command(target_state)

                if i < len(self.recorded_data) - 1:
                    next_time = float(self.recorded_data[i + 1].get("timestamp", 0))
                    current_time = float(data.get("timestamp", 0))
                    time.sleep(max(0.1, next_time - current_time))
                else:
                    time.sleep(0.1)

            except Exception as e:
                print(f"Playback error: {e}")
                break

        self.playing = False
        self.root.after(0, lambda: messagebox.showinfo("Playback", "Playback completed"))
    
    # Control methods
    def on_slider_change(self, index, value):
        self.servo_labels[index].config(text=f"{float(value):.1f}°")
        
    def set_single_servo(self, index):
        if not self.connected:
            messagebox.showwarning("Connection", "Connect to the robotic arm first.")
            return

        if self.emergency_stop_active:
            messagebox.showwarning("Emergency Stop", "Emergency stop is active!")
            return

        try:
            angle = float(self.slider_vars[index].get())
        except (ValueError, TypeError):
            messagebox.showerror("Invalid Angle", f"Invalid angle for Servo {index + 1}.")
            return

        lo, hi = self.servo_limits[index]
        angle = max(lo, min(hi, angle))

        # S6 exists in the GUI but is not a physical ESP32 servo.
        if index == 5:
            self.current_angles[5] = angle
            self.target_angles[5] = angle
            self.update_servo_display(5, angle)
            if self.debug_mode:
                print(f"UI-only S6 → {angle:.1f}°")
            return

        # Change only the selected target. All other targets are preserved.
        self.target_angles[index] = angle

        # ESP32 controls S1-S5 only.
        physical_angles = self.target_angles[:5]
        cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical_angles)
        self.send_command(cmd)

        self.update_servo_display(index, angle)

        if self.debug_mode:
            print(f"SETTING S{index + 1} ONLY → {angle:.1f}°")
            print(f"SENDING: {cmd}")
        
    def move_to_preset(self, angles, name):
        if self.emergency_stop_active:
            messagebox.showwarning("Emergency Stop", "Emergency stop is active!")
            return
        if not self.connected:
            messagebox.showwarning("Connection", "Connect to the robotic arm first.")
            return
        if len(angles) < 6:
            messagebox.showerror("Preset Error", "Preset must contain six values.")
            return

        self.target_angles = [float(a) for a in angles[:6]]

        for i in range(6):
            lo, hi = self.servo_limits[i]
            self.target_angles[i] = max(lo, min(hi, self.target_angles[i]))
            self.slider_vars[i].set(self.target_angles[i])
            self.servo_labels[i].config(text=f"{self.target_angles[i]:.1f}°")

        physical = self.target_angles[:5]
        cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
        self.send_command(cmd)

        if self.debug_mode:
            print(f"Moving to preset {name}: {cmd}")
        
    def emergency_stop(self):
        self.send_command("EMERGENCY_STOP")
        self.emergency_stop_active = True
        self.joystick_control_enabled = False
        if hasattr(self, 'joystick_active_label'):
            self.joystick_active_label.config(text="DISABLED", foreground='red')
        print("DEBUG: Emergency stop activated")
        
    def reset_emergency_stop(self):
        self.send_command("RESET_ESTOP")
        self.emergency_stop_active = False
        print("DEBUG: Emergency stop reset")
        
    def send_gripper_command(self, command):
        # Normalize command for consistent industrial logic
        # ON (Close) -> Sends 'CLOSE' to Arduino -> Arduino sets HIGH (CLOSED) -> Serial returns 'CLOSED' -> UI stays ON
        # OFF (Open) -> Sends 'OPEN' to Arduino -> Arduino sets LOW (OPEN) -> Serial returns 'OPEN' -> UI stays OFF
        cmd_upper = command.upper()
        if "ON" in cmd_upper or "CLOSE" in cmd_upper:
            arduino_cmd = "CLOSE"
            display_state = "ON"
        else:
            arduino_cmd = "OPEN"
            display_state = "OFF"
            
        self.send_command(f"GRIPPER|{arduino_cmd}")
        self.gripper_state.set(display_state)
        print(f"DEBUG: Relay command: {arduino_cmd} (Display: {display_state})")
        
    def toggle_gripper(self):
        if self.gripper_state.get() == "ON":
            self.send_gripper_command("OFF")
        else:
            self.send_gripper_command("ON")
    
    def set_movement_speed(self):
        self.send_command(f"SET_SPEED|{self.movement_speed}")
        
    def set_movement_mode(self, mode):
        self.movement_mode = mode
        self.send_command(f"SET_MODE|{mode}")
        print(f"DEBUG: Movement mode set to {mode}")
        
    def go_home(self):
        if self.emergency_stop_active:
            messagebox.showwarning("Emergency Stop", "Emergency stop is active!")
            return

        if not self.connected:
            messagebox.showwarning(
                "Connection",
                "Connect to the robotic arm first."
            )
            return

        home_angles = [0.0, 110.0, 125.0, 90.0, 80.0, 90.0]

        self.target_angles = home_angles.copy()

        for i in range(6):
            self.slider_vars[i].set(home_angles[i])
            self.servo_labels[i].config(
                text=f"{home_angles[i]:.1f}°"
            )

        # ESP32 handles S1-S5
        self.send_command("HOME")

        if self.debug_mode:
            print(
                "HOME sent: "
                "S1=0°, S2=110°, S3=125°, S4=90°, S5=80°, S6=90° UI-only"
            )
        
    # Joystick methods
    def init_joystick(self):
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.joystick_connected = True
                
                joystick_name = self.joystick.get_name()
                self.joystick_status_label.config(text="Connected", foreground='green')
                self.joystick_name_label.config(text=f"Device: {joystick_name}")
                
                print(f"Joystick initialized: {joystick_name}")
                print(f"  Axes: {self.joystick.get_numaxes()}")
                print(f"  Buttons: {self.joystick.get_numbuttons()}")
                
                self.joystick_running = True
                self.joystick_thread = threading.Thread(target=self.joystick_monitor_loop)
                self.joystick_thread.daemon = True
                self.joystick_thread.start()
                
                self.status_labels['joystick_label'].config(text="CONNECTED")
                
            else:
                print("No joystick found")
                self.joystick_connected = False
                
        except Exception as e:
            print(f"Error initializing joystick: {e}")
            self.joystick_connected = False
    
    def joystick_monitor_loop(self):
        last_button_states = [0] * 8  # Track button states for 8 buttons
        axis_values = [0.0] * 4
        
        while self.joystick_running and self.joystick:
            try:
                pygame.event.pump()
                current_time = time.time()
                
                for i in range(min(4, self.joystick.get_numaxes())):
                    raw_value = self.joystick.get_axis(i)
                    
                    if abs(raw_value) < self.deadzone_var.get():
                        processed_value = 0
                    else:
                        if raw_value > 0:
                            processed_value = (raw_value - self.deadzone_var.get()) / (1 - self.deadzone_var.get())
                        else:
                            processed_value = (raw_value + self.deadzone_var.get()) / (1 - self.deadzone_var.get())
                    
                    axis_values[i] = processed_value
                    self.update_axis_display(i, processed_value)
                
                # Check for button presses (B0, B1, B2, B3)
                for i in [0, 1, 2, 3]:  # Only check the buttons we care about
                    if i < self.joystick.get_numbuttons():
                        current_state = self.joystick.get_button(i)
                        
                        # Update button visualization
                        button_index = [0, 1, 2, 3].index(i) if i in [0, 1, 2, 3] else None
                        if button_index is not None and button_index < len(self.button_canvases):
                            color = 'green' if current_state else 'red'
                            self.button_canvases[button_index].itemconfig(self.button_lights[button_index], fill=color)
                        
                        # Check for button press (rising edge)
                        if current_state and not last_button_states[i]:
                            self.handle_joystick_button(i)
                        
                        last_button_states[i] = current_state
                
                if self.joystick_control_enabled and self.connected and not self.emergency_stop_active:
                    if current_time - self.last_joystick_update >= self.joystick_update_interval:
                        axis_changed = False
                        for i in range(4):
                            if abs(axis_values[i]) > 0.01:
                                self.handle_joystick_axis(i, axis_values[i])
                                axis_changed = True
                        
                        if axis_changed:
                            self.send_joystick_position()
                            self.last_joystick_update = current_time
                
                time.sleep(0.02)
                
            except Exception as e:
                print(f"Error in joystick loop: {e}")
                time.sleep(0.1)
    
    def update_axis_display(self, axis_idx, value):
        if axis_idx < len(self.axis_labels):
            canvas, bar, label = self.axis_labels[axis_idx]
            
            center_x = 100
            bar_width = int(abs(value) * 90)
            
            if value < 0:
                x1 = center_x - bar_width
                x2 = center_x
            else:
                x1 = center_x
                x2 = center_x + bar_width
            
            canvas.coords(bar, x1, 5, x2, 15)
            label.config(text=f"{value:.3f}")
    
    def handle_joystick_axis(self, axis_idx, value):
        if axis_idx == 0:
            servo_idx = 0
            max_change = 2.0
        elif axis_idx == 1:
            # INVERTED CONTROL: positive value = decrement, negative value = increment
            servo_idx = 1
            max_change = 2.0
            # Invert the value
            value = -value
        elif axis_idx == 2:
            servo_idx = 2
            max_change = 2.0
        elif axis_idx == 3:
            servo_idx = 3
            max_change = 2.0
        else:
            return
        
        angle_change = value * self.sens_var.get() * max_change
        
        self.smoothing_buffers[servo_idx].append(angle_change)
        
        if len(self.smoothing_buffers[servo_idx]) > 0:
            smoothed_change = sum(self.smoothing_buffers[servo_idx]) / len(self.smoothing_buffers[servo_idx])
        else:
            smoothed_change = angle_change
        
        new_angle = self.target_angles[servo_idx] + smoothed_change
        
        min_limit, max_limit = self.servo_limits[servo_idx]
        new_angle = max(min_limit, min(max_limit, new_angle))
        
        self.target_angles[servo_idx] = new_angle
        
        self.root.after(0, lambda idx=servo_idx, angle=new_angle: 
                       self.slider_vars[idx].set(angle))
    
    def handle_joystick_button(self, button_idx):
        print(f"Joystick Button {button_idx} pressed")
        
        if button_idx == 0:
            # B0: Go to Home Position
            self.go_home()
        elif button_idx == 1:
            # B1: Wrist Roll INCREASE
            self.adjust_wrist_roll(self.wrist_increment)
        elif button_idx == 2:
            # B2: Wrist Roll DECREASE
            self.adjust_wrist_roll(-self.wrist_increment)
        elif button_idx == 3:
            # B3: Toggle Gripper (Relay)
            self.toggle_gripper()
    
    def adjust_wrist_roll(self, amount):
        servo_idx = 4  # Wrist Roll is servo 5 (index 4)
        new_angle = self.target_angles[servo_idx] + amount
        
        min_limit, max_limit = self.servo_limits[servo_idx]
        new_angle = max(min_limit, min(max_limit, new_angle))
        
        self.target_angles[servo_idx] = new_angle
        self.slider_vars[servo_idx].set(new_angle)
        
        if self.joystick_control_enabled and self.connected:
            self.send_joystick_position()
    
    def send_joystick_position(self):
        if not self.connected or not self.joystick_control_enabled or self.emergency_stop_active:
            return

        physical = self.target_angles[:5]
        angles_str = "|".join(f"{angle:.1f}" for angle in physical)
        cmd = f"MOVE|{angles_str}"

        if self.debug_mode:
            print(f"JOYSTICK: {cmd}")

        self.send_command(cmd)
    
    def toggle_joystick_control(self):
        if not self.joystick_connected:
            messagebox.showwarning("Joystick", "No joystick connected!")
            return
            
        if not self.connected:
            messagebox.showwarning("Connection", "Not connected to arm!")
            return
        
        self.joystick_control_enabled = not self.joystick_control_enabled
        
        if self.joystick_control_enabled:
            self.enable_joystick_btn.config(text="Disable Joystick Control")
            self.joystick_active_label.config(text="ENABLED", foreground='green')
            
            self.target_angles = self.current_angles.copy()
            
            for buffer in self.smoothing_buffers:
                buffer.clear()
                for _ in range(5):
                    buffer.append(0)
            
            print("Joystick control ENABLED")
        else:
            self.enable_joystick_btn.config(text="Enable Joystick Control")
            self.joystick_active_label.config(text="DISABLED", foreground='red')
            print("Joystick control DISABLED")
    
    # Additional methods
    def move_to_cell(self, cell_number):
        if self.emergency_stop_active:
            messagebox.showwarning("Emergency Stop", "Emergency stop is active!")
            return
        if not self.connected:
            messagebox.showwarning("Connection", "Connect to the robotic arm first.")
            return
        if cell_number not in self.cell_positions:
            messagebox.showwarning("Empty Cell", f"Cell {cell_number} has no saved position.")
            return

        angles = list(self.cell_positions[cell_number])
        if len(angles) < 6:
            messagebox.showerror("Cell Error", "Saved cell must contain six values.")
            return

        self.target_angles = [float(a) for a in angles[:6]]

        for i in range(6):
            lo, hi = self.servo_limits[i]
            self.target_angles[i] = max(lo, min(hi, self.target_angles[i]))
            self.slider_vars[i].set(self.target_angles[i])
            self.servo_labels[i].config(text=f"{self.target_angles[i]:.1f}°")

        physical = self.target_angles[:5]
        cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
        self.send_command(cmd)

        if self.debug_mode:
            print(f"Moving to Cell {cell_number}: {cmd}")
        
    def on_cell_selected(self):
        cell_num = self.selected_cell_idx.get()
        self.cell_title_label.config(text=f"Editing Cell {cell_num}")
        
        # Load data into entry fields
        if cell_num in self.cell_positions:
            angles = self.cell_positions[cell_num]
            for i in range(6):
                self.cell_edit_vars[i].set(str(angles[i]))
        else:
            for i in range(6):
                self.cell_edit_vars[i].set("")
                
    def teach_selected_cell(self):
        cell_num = self.selected_cell_idx.get()
        # Round current angles to 1 decimal place
        angles = [round(a, 1) for a in self.current_angles]
        self.cell_positions[cell_num] = angles
        
        # Update entry fields
        for i in range(6):
            self.cell_edit_vars[i].set(str(angles[i]))
            
        messagebox.showinfo("Success", f"Arm position copied to Cell {cell_num}")

    def update_selected_cell_data(self):
        cell_num = self.selected_cell_idx.get()
        try:
            new_angles = []
            for i in range(6):
                val = float(self.cell_edit_vars[i].get())
                # Validate against limits
                min_l, max_l = self.servo_limits[i]
                if not (min_l <= val <= max_l):
                    raise ValueError(f"S{i+1} value {val} is outside limits ({min_l}-{max_l})")
                new_angles.append(val)
                
            self.cell_positions[cell_num] = new_angles
            messagebox.showinfo("Success", f"Cell {cell_num} updated successfully")
            
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input: {str(e)}")

    def delete_selected_cell_data(self):
        cell_num = self.selected_cell_idx.get()
        if cell_num in self.cell_positions:
            if messagebox.askyesno("Confirm", f"Are you sure you want to clear Cell {cell_num}?"):
                del self.cell_positions[cell_num]
                self.on_cell_selected() # Refresh display
        else:
            messagebox.showwarning("Warning", f"Cell {cell_num} is already empty")
        
    def save_cell_positions(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")],
            initialfile=f"cell_positions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        if filename:
            with open(filename, 'w') as f:
                json.dump(self.cell_positions, f, indent=4)
            messagebox.showinfo("Saved", f"Cell positions saved to {filename}")
            
    def load_cell_positions(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json")]
        )
        if filename:
            with open(filename, 'r') as f:
                loaded_data = json.load(f)
                # Convert keys back to integers if they were saved as strings (JSON standard)
                self.cell_positions = {int(k): v for k, v in loaded_data.items()}
            self.on_cell_selected()
            messagebox.showinfo("Loaded", f"Cell positions loaded from {filename}")
            
    def get_current_position(self):
        self.send_command("GET_POSITION")
        
    def send_current_position(self):
        if not self.connected:
            messagebox.showwarning("Connection", "Connect to the robotic arm first.")
            return

        physical = self.target_angles[:5]
        cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
        self.send_command(cmd)

        if self.debug_mode:
            print(f"Sending physical position: {cmd}")
        
    def save_position(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt")],
            initialfile=f"position_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        )
        
        if filename:
            with open(filename, 'w') as f:
                f.write(f"Position saved at {datetime.now()}\n")
                f.write("Servo angles:\n")
                for i, angle in enumerate(self.current_angles):
                    f.write(f"  Servo {i+1}: {angle:.1f}°\n")
            messagebox.showinfo("Saved", f"Position saved to {filename}")
            
    def refresh_system_status(self):
        self.status_labels['uptime_label'].config(text=f"{self.system_uptime/1000:.1f}s")
        self.status_labels['move_count_label'].config(text=str(self.movement_count))
        self.status_labels['log_size_label'].config(text=str(self.log_size))
        self.status_labels['system_state_label'].config(
            text="RUNNING" if not self.emergency_stop_active else "ESTOP")
        self.status_labels['estop_label'].config(
            text="ACTIVE" if self.emergency_stop_active else "INACTIVE")
        self.status_labels['conn_label'].config(
            text="CONNECTED" if self.connected else "DISCONNECTED")
        self.status_labels['speed_label'].config(text="100 (Constant)")
        self.status_labels['control_label'].config(
            text="SMOOTH" if self.movement_mode == "smooth" else "DIRECT")
        self.status_labels['joystick_label'].config(
            text="CONNECTED" if self.joystick_connected else "DISCONNECTED")
            
    def update_system_status(self):
        self.refresh_system_status()
        self.root.after(1000, self.update_system_status)
        
    def get_system_status(self):
        # ESP32 firmware provides GET_STATUS.
        self.send_command("GET_STATUS")
        
    def run_diagnostic(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to arm")
            return

        messagebox.showinfo("Diagnostic", "Starting diagnostic test...")
        original_angles = self.target_angles.copy()

        test_positions = [
            [90, 90, 90, 90, 90],
            [60, 110, 120, 80, 120],
            [120, 70, 150, 110, 150],
        ]

        def run_test_sequence(index=0):
            if not self.connected:
                return

            if index < len(test_positions):
                physical = test_positions[index]
                cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
                self.send_command(cmd)
                self.target_angles[:5] = [float(a) for a in physical]
                self.root.after(3000, lambda i=index + 1: run_test_sequence(i))
            else:
                physical = original_angles[:5]
                cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
                self.send_command(cmd)
                self.target_angles = original_angles.copy()
                self.root.after(
                    2000,
                    lambda: messagebox.showinfo(
                        "Diagnostic",
                        "Diagnostic test completed successfully!"
                    )
                )

        run_test_sequence()
        
    def calibrate_servos(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to arm")
            return

        messagebox.showinfo("Calibration", "Starting servo calibration...")
        original_s6 = self.target_angles[5]

        calibration_positions = [
            [90, 90, 90, 90, 90],
            [80, 100, 100, 100, 100],
            [100, 80, 120, 80, 120],
            [90, 90, 90, 90, 90],
        ]

        def run_calibration(index=0):
            if not self.connected:
                return

            if index < len(calibration_positions):
                physical = calibration_positions[index]
                cmd = "MOVE|" + "|".join(f"{a:.1f}" for a in physical)
                self.send_command(cmd)
                self.target_angles[:5] = [float(a) for a in physical]
                self.root.after(2000, lambda i=index + 1: run_calibration(i))
            else:
                self.send_command("HOME")
                self.target_angles[:5] = [90.0] * 5
                self.target_angles[5] = original_s6
                self.root.after(
                    1000,
                    lambda: messagebox.showinfo(
                        "Calibration",
                        "Servo calibration completed!"
                    )
                )

        run_calibration()
        
    def start_logging(self):
        self.send_command("START_LOGGING")
        
    def stop_logging(self):
        self.send_command("STOP_LOGGING")
        
    def get_log_size(self):
        self.send_command("GET_LOG_SIZE")
        
    def clear_log(self):
        self.send_command("CLEAR_LOG")
        
    def export_csv(self):
        if not self.recorded_data:
            messagebox.showwarning("No Data", "No recorded data to export")
            return
            
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"arm_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        if filename:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'Gripper'])
                for data in self.recorded_data:
                    writer.writerow([
                        data['timestamp'],
                        data['angles'][0],
                        data['angles'][1],
                        data['angles'][2],
                        data['angles'][3],
                        data['angles'][4],
                        data['angles'][5],
                        data['gripper']
                    ])
            messagebox.showinfo("Exported", f"Data exported to {filename}")
            
    def test_connection(self):
        if not self.connected:
            messagebox.showerror("Error", "Not connected to arm")
            return
            
        self.send_command("GET_STATUS")
        messagebox.showinfo("Test", "Connection test sent. Check console for response.")
        
    def toggle_debug(self):
        self.debug_mode = not self.debug_mode
        if self.debug_mode:
            self.debug_btn.config(text="Debug OFF")
            if self.connected:
                self.send_command("DEBUG_ON")
            print("Debug mode ON")
        else:
            self.debug_btn.config(text="Debug ON")
            if self.connected:
                self.send_command("DEBUG_OFF")
            print("Debug mode OFF")
    
    def update_all_servo_displays(self, angles):
        for i in range(min(6, len(angles))):
            self.update_servo_display(i, float(angles[i]))

    def update_servo_display(self, index, angle):
        self.servo_labels[index].config(text=f"{angle:.1f}°")
        self.slider_vars[index].set(angle)
    
    def update_increment_display(self):
        self.wrist_increment = self.increment_var.get()
        self.increment_label.config(text=f"{self.wrist_increment:.0f}°")
    
    def on_closing(self):
        self.joystick_running = False
        self.connected = False
        self.recording = False
        self.playing = False
        
        if self.serial_port:
            self.serial_port.close()
            
        pygame.quit()
        self.root.destroy()

def main():
    root = tk.Tk()
    
    style = ttk.Style()
    style.theme_use('clam')
    
    style.configure('Emergency.TButton', 
                   foreground='white', 
                   background='red', 
                   font=('Arial', 10, 'bold'),
                   padding=10)
    
    style.configure('Reset.TButton', 
                   foreground='white', 
                   background='green', 
                   font=('Arial', 10, 'bold'),
                   padding=5)
    
    app = IndustrialRoboticArmController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()