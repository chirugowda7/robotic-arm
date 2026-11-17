import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import json
import logging
import os
import glob
import re
from datetime import datetime
from enum import Enum

# --- Industrial Logging Setup ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler(f"abb_rapid_log_{datetime.now().strftime('%Y%m%d')}.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("ABBRAPID")

class RobotState(Enum):
    IDLE = 0
    RUNNING = 1
    SIMULATING = 2
    ERROR = 3
    ESTOP = 4
    DISCONNECTED = 5

class ABBRAPIDProgrammer:
    def __init__(self, root):
        self.root = root
        self.root.title("ABB Robotics - RAPID & Block Programmer")
        
        # Fit to screen logic
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = min(1300, int(screen_width * 0.9))
        height = min(900, int(screen_height * 0.9))
        self.root.geometry(f"{width}x{height}")
        try:
            self.root.state('zoomed') # Maximize on Windows
        except:
            pass
        
        # Connection & State
        self.serial_port = None
        self.connected = False
        self.simulation_mode = tk.BooleanVar(value=False)
        self.robot_state = RobotState.DISCONNECTED
        self.current_angles = [0.0, 90.0, 175.0, 115.0, 145.0, 90.0]
        
        # Cell positions (Restored from original hardware specifications)
        self.cell_positions = {
            "Home": [0.0, 113.1, 148.9, 81.1, 139.3, 180],
            "Safe": [45, 90, 173, 119, 75, 90],
            0: [0, 90, 175, 115, 145, 90],    
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
        
        # Programming state
        self.program_sequence = [] 
        self.is_running = False
        self.stop_requested = False
        self.movement_complete_event = threading.Event()
        
        self.setup_ui()
        logger.info("ABB RAPID Application initialized")
        
    def setup_ui(self):
        # --- Top System Bar ---
        top_frame = ttk.Frame(self.root, padding=5)
        top_frame.pack(fill='x')
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(top_frame, text="Controller Communication", padding=10)
        conn_frame.pack(side='left', fill='y', padx=5)
        
        ttk.Label(conn_frame, text="Port:").pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=12)
        self.port_combo.pack(side='left', padx=5)
        
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).pack(side='left', padx=2)
        
        self.conn_btn = ttk.Button(conn_frame, text="Connect Controller", command=self.toggle_connection)
        self.conn_btn.pack(side='left', padx=10)
        
        self.status_led = tk.Canvas(conn_frame, width=20, height=20, highlightthickness=0)
        self.status_led.pack(side='left')
        self.led_circle = self.status_led.create_oval(2, 2, 18, 18, fill='red')
        
        self.refresh_ports() # Initial scan
        
        # Simulation Frame
        sim_frame = ttk.LabelFrame(top_frame, text="Virtual Controller", padding=10)
        sim_frame.pack(side='left', fill='y', padx=5)
        ttk.Checkbutton(sim_frame, text="Enable Simulation", variable=self.simulation_mode).pack(side='left', padx=5)

        # Emergency Stop
        ttk.Button(top_frame, text="🛑 MOTOR OFF (ESTOP)", command=self.emergency_stop, style='Emergency.TButton').pack(side='right', padx=5, pady=5)

        # --- Main Layout ---
        paned_window = ttk.PanedWindow(self.root, orient='horizontal')
        paned_window.pack(fill='both', expand=True, padx=10, pady=5)

        # 1. LEFT: Instruction Palette & Manual Control (with Scrollbar)
        sidebar_canvas = tk.Canvas(paned_window, highlightthickness=0)
        sidebar_scrollbar = ttk.Scrollbar(paned_window, orient="vertical", command=sidebar_canvas.yview)
        left_sidebar = ttk.Frame(sidebar_canvas)
        
        sidebar_canvas.create_window((0, 0), window=left_sidebar, anchor="nw")
        sidebar_canvas.configure(yscrollcommand=sidebar_scrollbar.set)
        
        paned_window.add(sidebar_canvas, weight=1)

        def on_sidebar_configure(event):
            sidebar_canvas.configure(scrollregion=sidebar_canvas.bbox("all"))
            sidebar_canvas.itemconfig(sidebar_canvas.create_window((0, 0), window=left_sidebar, anchor="nw"), width=event.width)

        sidebar_canvas.bind('<Configure>', on_sidebar_configure)

        # Instruction Palette
        palette_frame = ttk.LabelFrame(left_sidebar, text="RAPID Instructions", padding=10)
        palette_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Label(palette_frame, text="MOTION:", font=('Arial', 9, 'bold')).pack(anchor='w', pady=(5, 2))
        ttk.Button(palette_frame, text="MoveAbsJ [Home]", command=lambda: self.add_block('MoveAbsJ', 'Home')).pack(fill='x', pady=2)
        ttk.Button(palette_frame, text="MoveJ [Safe]", command=lambda: self.add_block('MoveJ', 'SafePos')).pack(fill='x', pady=2)
        ttk.Button(palette_frame, text="SetPos [Direct]", command=lambda: self.add_block('SetPos', 'Direct')).pack(fill='x', pady=2)
        
        cell_grid = ttk.Frame(palette_frame)
        cell_grid.pack(pady=5)
        for i in range(1, 10):
            r, c = (i-1)//3, (i-1)%3
            btn = ttk.Button(cell_grid, text=f"p{i}", width=4, 
                            command=lambda n=i: self.add_block('MoveJ', f"target_p{n}"))
            btn.grid(row=r, column=c, padx=2, pady=2)
            
        ttk.Label(palette_frame, text="I/O:", font=('Arial', 9, 'bold')).pack(anchor='w', pady=(5, 2))
        ttk.Button(palette_frame, text="SetDO [GripperOpen]", command=lambda: self.add_block('SetDO', 'Open')).pack(fill='x', pady=2)
        ttk.Button(palette_frame, text="ResetDO [GripperClose]", command=lambda: self.add_block('ResetDO', 'Close')).pack(fill='x', pady=2)
        
        ttk.Label(palette_frame, text="WAIT:", font=('Arial', 9, 'bold')).pack(anchor='w', pady=(5, 2))
        self.delay_var = tk.DoubleVar(value=3)
        ttk.Spinbox(palette_frame, from_=0.1, to=60.0, increment=0.1, textvariable=self.delay_var).pack(fill='x', pady=2)
        ttk.Button(palette_frame, text="Add WaitTime", command=lambda: self.add_block('WaitTime', self.delay_var.get())).pack(fill='x', pady=2)

        # Manual Joint Control (Move one servo at a time)
        manual_frame = ttk.LabelFrame(left_sidebar, text="Manual Joint Control", padding=10)
        manual_frame.pack(fill='x', padx=5, pady=5)
        
        self.joint_sliders = []
        for i in range(6):
            f = ttk.Frame(manual_frame)
            f.pack(fill='x', pady=2)
            ttk.Label(f, text=f"J{i+1}:", width=4).pack(side='left')
            slider = ttk.Scale(f, from_=0, to=180, orient='horizontal', command=lambda v, idx=i: self.manual_move(idx, v))
            slider.set(self.current_angles[i])
            slider.pack(side='left', fill='x', expand=True)
            self.joint_sliders.append(slider)
        
        # Teach Features
        ttk.Separator(manual_frame, orient='horizontal').pack(fill='x', pady=10)
        ttk.Button(manual_frame, text="📸 TEACH POSITION (Record)", command=self.teach_position).pack(fill='x', pady=5)
        ttk.Button(manual_frame, text="📸 TEACH SetPos (No Base)", command=lambda: self.teach_position(as_setpos=True)).pack(fill='x', pady=5)
        ttk.Button(manual_frame, text="⌨️ SET CUSTOM POSE (Manual)", command=self.set_custom_pose_dialog).pack(fill='x', pady=5)

        # 2. MIDDLE: Routine (Block View)
        seq_container = ttk.LabelFrame(paned_window, text="Main Routine (Blocks)", padding=10)
        paned_window.add(seq_container, weight=2)
        
        self.seq_listbox = tk.Listbox(seq_container, font=('Consolas', 11), selectmode='single', bg='#ffffff')
        self.seq_listbox.pack(side='left', fill='both', expand=True)
        
        scrollbar = ttk.Scrollbar(seq_container, orient="vertical", command=self.seq_listbox.yview)
        scrollbar.pack(side='right', fill='y')
        self.seq_listbox.config(yscrollcommand=scrollbar.set)

        # 3. RIGHT: RAPID Editor (Code View)
        code_container = ttk.Frame(paned_window)
        paned_window.add(code_container, weight=2)
        
        code_label_frame = ttk.LabelFrame(code_container, text="RAPID Module Editor", padding=10)
        code_label_frame.pack(fill='both', expand=True)
        
        self.code_text = tk.Text(code_label_frame, font=('Consolas', 10), bg='#2b2b2b', fg='#a9b7c6', padx=10, pady=10)
        self.code_text.pack(fill='both', expand=True)
        
        sync_frame = ttk.Frame(code_label_frame)
        sync_frame.pack(fill='x', pady=5)
        ttk.Button(sync_frame, text="🔄 RAPID to Blocks", command=self.sync_rapid_to_blocks).pack(side='left', padx=5)
        ttk.Button(sync_frame, text="🔄 Blocks to RAPID", command=self.update_code_preview).pack(side='left', padx=5)

        # --- Bottom Control Bar ---
        btn_frame = ttk.Frame(self.root, padding=10)
        btn_frame.pack(fill='x')
        
        ttk.Button(btn_frame, text="Delete Instruction", command=self.remove_block).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Clear Module", command=self.clear_program).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Export RAPID (.mod)", command=self.save_program).pack(side='left', padx=5)
        
        ttk.Separator(btn_frame, orient='vertical').pack(side='left', fill='y', padx=20)
        
        self.run_btn = ttk.Button(btn_frame, text="▶️ START PROGRAM", style='Run.TButton', command=self.run_program)
        self.run_btn.pack(side='left', padx=5, ipadx=20)
        
        self.stop_btn = ttk.Button(btn_frame, text="🛑 STOP CYCLE", command=self.stop_program)
        self.stop_btn.pack(side='left', padx=5)

        style = ttk.Style()
        style.configure('Run.TButton', font=('Arial', 10, 'bold'), foreground='#ff6a00')
        style.configure('Emergency.TButton', font=('Arial', 10, 'bold'), foreground='red')
        
    def set_custom_pose_dialog(self):
        """Open a dialog to manually enter exact angles for all joints."""
        dialog = tk.Toplevel(self.root)
        dialog.title("Manual Pose Input")
        dialog.geometry("350x450")
        dialog.transient(self.root)
        dialog.grab_set()

        vars = []
        for i in range(6):
            f = ttk.Frame(dialog, padding=10)
            f.pack(fill='x')
            ttk.Label(f, text=f"Joint {i+1} Angle (0-180):", width=20).pack(side='left')
            v = tk.StringVar(value=f"{self.current_angles[i]:.1f}")
            vars.append(v)
            ttk.Entry(f, textvariable=v, width=10).pack(side='right')

        def apply_pose(add_block=False):
            try:
                angles = [float(v.get()) for v in vars]
                if all(0 <= a <= 180 for a in angles):
                    self.current_angles = angles
                    for i in range(6): self.joint_sliders[i].set(angles[i])
                    if self.connected:
                        angles_str = "|".join(f"{a:.1f}" for a in angles)
                        self.send_command(f"MOVE|{angles_str}")
                    if add_block: self.add_block('MoveJ', f"taught_{angles}")
                    dialog.destroy()
                else: messagebox.showerror("Limit Error", "Angles must be 0-180")
            except ValueError: messagebox.showerror("Input Error", "Enter valid numbers")

        btn_f = ttk.Frame(dialog, padding=20)
        btn_f.pack(fill='x')
        ttk.Button(btn_f, text="Move Now", command=lambda: apply_pose(False)).pack(side='left', expand=True, fill='x', padx=5)
        ttk.Button(btn_f, text="Add to Routine", command=lambda: apply_pose(True)).pack(side='left', expand=True, fill='x', padx=5)

    def manual_move(self, joint_idx, value):
        self.current_angles[joint_idx] = float(value)
        if self.connected:
            angles_str = "|".join(f"{a:.1f}" for a in self.current_angles)
            self.send_command(f"MOVE|{angles_str}")

    def teach_position(self, as_setpos=False):
        taught_angles = list(self.current_angles)
        btype = 'SetPos' if as_setpos else 'MoveJ'
        self.add_block(btype, f"taught_{taught_angles}")

    def add_block(self, btype, params=None):
        block = {'type': btype, 'params': params}
        self.program_sequence.append(block)
        self.refresh_listbox()
        self.update_code_preview()
        
    def remove_block(self):
        selection = self.seq_listbox.curselection()
        if selection:
            self.program_sequence.pop(selection[0])
            self.refresh_listbox()
            self.update_code_preview()
            
    def clear_program(self):
        if messagebox.askyesno("Confirm", "Clear routine?"):
            self.program_sequence = []
            self.refresh_listbox()
            self.update_code_preview()
            
    def refresh_listbox(self):
        self.seq_listbox.delete(0, tk.END)
        for i, block in enumerate(self.program_sequence):
            self.seq_listbox.insert(tk.END, f"{i+1:03d}: {block['type']} {block['params']};")

    def update_code_preview(self):
        self.code_text.delete('1.0', tk.END)
        code = "MODULE MainModule\n\n  PROC main()\n"
        for block in self.program_sequence:
            btype, params = block['type'], str(block['params'])
            if btype == 'MoveAbsJ': code += f"    MoveAbsJ {params}, v100, fine, tool0;\n"
            elif btype == 'MoveJ': 
                if "taught_" in params:
                    angles = params.replace("taught_", "")
                    code += f"    MoveJ {angles}, v500, z50, tool0; ! Taught\n"
                else: code += f"    MoveJ {params}, v500, z50, tool0;\n"
            elif btype == 'SetPos': code += f"    SetPos Home_NoBase, v500, fine, tool0;\n"
            elif btype == 'SetDO': code += f"    SetDO doGripper, 1;\n"
            elif btype == 'ResetDO': code += f"    ResetDO doGripper, 0;\n"
            elif btype == 'WaitTime': code += f"    WaitTime {params};\n"
        code += "  ENDPROC\n\nENDMODULE"
        self.code_text.insert('1.0', code)

    def sync_rapid_to_blocks(self):
        raw_code = self.code_text.get('1.0', tk.END)
        new_seq = []
        for line in raw_code.split('\n'):
            line = line.strip()
            if not line or any(x in line for x in ['PROC','MODULE','END','!']): continue
            try:
                if 'MoveAbsJ' in line: new_seq.append({'type':'MoveAbsJ','params':re.search(r'MoveAbsJ\s+(\w+)',line).group(1)})
                elif 'MoveJ' in line:
                    arr_match = re.search(r'\[(.*?)\]', line)
                    if arr_match: new_seq.append({'type':'MoveJ','params':f"taught_{[float(x) for x in arr_match.group(1).split(',')]}"})
                    else: new_seq.append({'type':'MoveJ','params':re.search(r'MoveJ\s+(\w+)',line).group(1)})
                elif 'SetDO' in line: new_seq.append({'type':'SetDO','params':'Open'})
                elif 'ResetDO' in line: new_seq.append({'type':'ResetDO','params':'Close'})
                elif 'WaitTime' in line: new_seq.append({'type':'WaitTime','params':float(re.search(r'WaitTime\s+([\d\.]+)',line).group(1))})
            except: pass
        if new_seq: self.program_sequence = new_seq; self.refresh_listbox()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_combo.set("COM3" if "COM3" in ports else ports[0])

    def toggle_connection(self):
        if not self.connected:
            port = self.port_var.get()
            if not port: return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(2)
                self.connected = True
                self.robot_state = RobotState.IDLE
                self.conn_btn.config(text="Disconnect Controller")
                self.status_led.itemconfig(self.led_circle, fill='green')
                threading.Thread(target=self.serial_reader, daemon=True).start()
                self.send_command("GET_POSITION")
            except Exception as e:
                self.cleanup_connection()
                messagebox.showerror("Error", f"Conn Failed: {e}")
        else: self.cleanup_connection()

    def serial_reader(self):
        buffer = ""
        while self.connected and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    buffer += self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self.process_line(line.strip())
            except: break
            time.sleep(0.01)
        self.root.after(0, self.update_ui_disconnected)

    def process_line(self, line):
        if not line: return
        if line.startswith("STATUS|"):
            try:
                parts = line.split('|')
                if len(parts) >= 7:
                    self.current_angles = [float(parts[i+1]) for i in range(6)]
                    if not self.is_running:
                        for i in range(6): self.joint_sliders[i].set(self.current_angles[i])
                if "IDLE" in line: self.movement_complete_event.set()
            except: pass
        elif "MOVEMENT|COMPLETE" in line: self.movement_complete_event.set()

    def cleanup_connection(self):
        self.connected = False
        if self.serial_port: 
            try: self.serial_port.close()
            except: pass
            self.serial_port = None
        self.root.after(0, self.update_ui_disconnected)

    def update_ui_disconnected(self):
        self.conn_btn.config(text="Connect Controller")
        self.status_led.itemconfig(self.led_circle, fill='red')

    def emergency_stop(self):
        self.stop_requested = True
        self.is_running = False
        if self.connected: self.send_command("EMERGENCY_STOP")
        messagebox.showwarning("ESTOP", "Emergency Stop Triggered")

    def run_program(self):
        if not self.connected and not self.simulation_mode.get(): return
        self.is_running, self.stop_requested = True, False
        self.run_btn.config(state='disabled')
        threading.Thread(target=self.execution_loop, daemon=True).start()

    def stop_program(self): self.stop_requested, self.is_running = True, False

    def wait_for_ack(self, timeout=15.0):
        if self.simulation_mode.get(): time.sleep(0.5); return True
        self.movement_complete_event.clear()
        start = time.time()
        while (time.time() - start) < timeout:
            if not self.is_running or self.stop_requested: return False
            if self.movement_complete_event.is_set(): return True
            time.sleep(0.05)
        return False

    def execute_staged_move(self, target_angles):
        if self.simulation_mode.get(): time.sleep(1.0); self.current_angles = list(target_angles); return True
        # Stage 1: Base Only
        staged = list(self.current_angles)
        staged[0] = target_angles[0]
        if not self.send_command(f"MOVE|{'|'.join(f'{a:.1f}' for a in staged)}"): return False
        if not self.wait_for_ack(15.0): return False
        time.sleep(2.0)
        # Stage 2: Rest
        if not self.send_command(f"MOVE|{'|'.join(f'{a:.1f}' for a in target_angles)}"): return False
        return self.wait_for_ack(15.0)

    def execute_reverse_move(self, target_angles):
        if self.simulation_mode.get(): time.sleep(1.0); self.current_angles = list(target_angles); return True
        for i in range(5, -1, -1):
            if not self.is_running or self.stop_requested: return False
            cmd = list(self.current_angles); cmd[i] = target_angles[i]
            if not self.send_command(f"MOVE|{'|'.join(f'{a:.1f}' for a in cmd)}"): return False
            if not self.wait_for_ack(10.0): return False
        return True

    def execution_loop(self):
        for i, block in enumerate(self.program_sequence):
            if not self.is_running or self.stop_requested: break
            self.root.after(0, lambda idx=i: self.highlight_step(idx))
            btype, params = block['type'], block['params']
            success = False
            if btype == 'MoveAbsJ': success = self.execute_staged_move(self.cell_positions["Home"])
            elif btype == 'MoveJ':
                target = None
                if "taught_" in str(params): target = list(eval(params.replace("taught_", "")))
                elif "target_p" in str(params):
                    match = re.search(r'target_p(\d+)', str(params))
                    if match: target = self.cell_positions.get(int(match.group(1)))
                elif params == 'SafePos': target = self.cell_positions["Safe"]
                if target: success = self.execute_staged_move(target)
            elif btype == 'SetPos':
                target = None
                if "taught_" in str(params): target = list(eval(params.replace("taught_", "")))
                elif "target_p" in str(params):
                    match = re.search(r'target_p(\d+)', str(params))
                    if match: target = self.cell_positions.get(int(match.group(1)))
                if not target: target = self.cell_positions["Home"]
                
                # s0 is not considered, make s0 the previous position
                target_angles = [self.current_angles[0]] + target[1:]
                
                if self.simulation_mode.get():
                    time.sleep(1.0)
                    self.current_angles = target_angles
                    success = True
                else:
                    cmd = f"MOVE|{'|'.join(f'{a:.1f}' for a in target_angles)}"
                    if self.send_command(cmd):
                        time.sleep(0.1) # Sync delay
                        success = self.wait_for_ack(20.0)
            elif btype == 'SetDO': success = self.send_command("GRIPPER|OPEN"); time.sleep(1.0)
            elif btype == 'ResetDO': success = self.send_command("GRIPPER|CLOSE"); time.sleep(1.0)
            elif btype == 'WaitTime': time.sleep(float(params)); success = True
            if not success and not self.simulation_mode.get(): break
        self.root.after(0, self.cleanup_execution)

    def highlight_step(self, idx):
        self.seq_listbox.selection_clear(0, tk.END)
        self.seq_listbox.selection_set(idx)
        self.seq_listbox.see(idx)

    def cleanup_execution(self):
        self.is_running = False
        self.run_btn.config(state='normal')
        self.seq_listbox.selection_clear(0, tk.END)

    def send_command(self, cmd):
        if self.simulation_mode.get(): logger.info(f"[SIM] {cmd}"); return True
        if self.serial_port and self.connected:
            try: self.serial_port.write((cmd + "\n").encode()); return True
            except: self.cleanup_connection(); return False
        return False

    def save_program(self):
        path = filedialog.asksaveasfilename(defaultextension=".mod", filetypes=[("RAPID", "*.mod")])
        if path:
            with open(path, 'w') as f: f.write(self.code_text.get('1.0', tk.END))

if __name__ == "__main__":
    root = tk.Tk()
    app = ABBRAPIDProgrammer(root)
    root.mainloop()