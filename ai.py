import tkinter as tk
from tkinter import ttk, messagebox
import cv2
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
import threading
import time
import numpy as np

class AIProgrammer:
    def __init__(self, root):
        self.root = root
        self.root.title("AI Sorting Robot - Color Detection")
        self.root.geometry("1400x900")
        
        # Connection & State
        self.serial_port = None
        self.connected = False
        self.simulation_mode = tk.BooleanVar(value=False)
        self.current_angles = [0.0, 90.0, 175.0, 115.0, 145.0, 90.0]
        
        self.positions = {
            "Home": [0.4, 100.0, 159.1, 98.7, 145, 90],
             
            "PickStand": [50, 90, 176, 126, 145, 90],
            "Pick": [50, 63, 176, 126, 145, 90],
            "SetPos": [90, 175, 115, 145, 90],
            "RedPlaceStand": [0, 80, 180, 118, 145, 90],
            "RedPlace": [0, 66, 180, 118, 145, 90],
            "YellowPlaceStand": [109, 80, 180, 122, 145, 90],
            "YellowPlace": [109, 66, 180, 122, 145, 90]
        }
        
        self.is_running = False
        self.stop_requested = False
        self.movement_complete_event = threading.Event()
        
        # Camera & Vision
        self.cap = None
        self.camera_running = False
        self.last_detected_color = "NONE"
        
        self.setup_ui()
        self.refresh_ports()
        
    def setup_ui(self):
        # Top Bar (Connection & Camera)
        top_frame = ttk.Frame(self.root, padding=5)
        top_frame.pack(fill='x')
        
        # Connection
        conn_frame = ttk.LabelFrame(top_frame, text="Controller Communication", padding=10)
        conn_frame.pack(side='left', fill='y', padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=10)
        self.port_combo.pack(side='left', padx=5)
        
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).pack(side='left', padx=2)
        self.conn_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.conn_btn.pack(side='left', padx=5)
        
        ttk.Checkbutton(conn_frame, text="Simulation Mode", variable=self.simulation_mode).pack(side='left', padx=10)
        
        # Camera
        cam_frame = ttk.LabelFrame(top_frame, text="Camera (External Port)", padding=10)
        cam_frame.pack(side='left', fill='y', padx=5)
        
        ttk.Label(cam_frame, text="Index:").pack(side='left')
        self.cam_index_var = tk.IntVar(value=0) # Default to 0 for default camera
        ttk.Entry(cam_frame, textvariable=self.cam_index_var, width=5).pack(side='left', padx=5)
        
        self.cam_btn = ttk.Button(cam_frame, text="Start Camera", command=self.toggle_camera)
        self.cam_btn.pack(side='left', padx=5)
        
        ttk.Label(cam_frame, text="Zoom:").pack(side='left', padx=(5,0))
        self.zoom_var = tk.DoubleVar(value=1.0)
        ttk.Scale(cam_frame, from_=1.0, to=4.0, variable=self.zoom_var, orient='horizontal').pack(side='left', padx=5)
        
        ttk.Button(top_frame, text="🛑 MOTOR OFF (ESTOP)", command=self.emergency_stop).pack(side='right', padx=10)
        
        # Layout
        paned = ttk.PanedWindow(self.root, orient='horizontal')
        paned.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Left Panel (Camera View)
        left_panel = ttk.Frame(paned)
        paned.add(left_panel, weight=3)
        
        cam_view_frame = ttk.LabelFrame(left_panel, text="Live Camera & Color Detection", padding=10)
        cam_view_frame.pack(fill='both', expand=True)
        
        self.cam_label = ttk.Label(cam_view_frame, text="Camera Offline", anchor='center', background="black", foreground="white")
        self.cam_label.pack(fill='both', expand=True)
        
        self.color_label = ttk.Label(cam_view_frame, text="DETECTED: NONE", font=('Arial', 18, 'bold'))
        self.color_label.pack(pady=10)
        
        # Right Panel (Controls & Teach)
        right_panel = ttk.Frame(paned)
        paned.add(right_panel, weight=1)
        
        manual_frame = ttk.LabelFrame(right_panel, text="Manual Joint Control", padding=10)
        manual_frame.pack(fill='x', pady=5)
        
        self.joint_sliders = []
        for i in range(6):
            f = ttk.Frame(manual_frame)
            f.pack(fill='x', pady=2)
            ttk.Label(f, text=f"J{i+1}:", width=4).pack(side='left')
            slider = ttk.Scale(f, from_=0, to=180, orient='horizontal', command=lambda v, idx=i: self.manual_move(idx, v))
            slider.set(self.current_angles[i])
            slider.pack(side='left', fill='x', expand=True)
            self.joint_sliders.append(slider)
            
        teach_frame = ttk.LabelFrame(right_panel, text="Teach Target Positions", padding=10)
        teach_frame.pack(fill='x', pady=5)
        
        ttk.Button(teach_frame, text="Teach Home (Initial)", command=lambda: self.teach_pos("Home")).pack(fill='x', pady=2)
        ttk.Button(teach_frame, text="Teach Pick Position", command=lambda: self.teach_pos("Pick")).pack(fill='x', pady=2)
        ttk.Button(teach_frame, text="Teach SetPos (Lift)", command=lambda: self.teach_pos("SetPos")).pack(fill='x', pady=2)
        ttk.Button(teach_frame, text="Teach Red Place", command=lambda: self.teach_pos("Red")).pack(fill='x', pady=2)
        ttk.Button(teach_frame, text="Teach Yellow Place", command=lambda: self.teach_pos("Yellow")).pack(fill='x', pady=2)
        
        run_frame = ttk.LabelFrame(right_panel, text="Auto Sequence Control", padding=10)
        run_frame.pack(fill='x', pady=10)
        
        self.run_btn = ttk.Button(run_frame, text="▶ START SORT CYCLE", command=self.start_cycle)
        self.run_btn.pack(fill='x', pady=5)
        
        self.stop_btn = ttk.Button(run_frame, text="⏹ STOP CYCLE", command=self.stop_cycle)
        self.stop_btn.pack(fill='x', pady=5)
        
        log_frame = ttk.LabelFrame(right_panel, text="Execution Log", padding=5)
        log_frame.pack(fill='both', expand=True, pady=5)
        
        self.log_text = tk.Text(log_frame, height=10, width=40, state='normal')
        self.log_text.pack(fill='both', expand=True)
        
    def log(self, msg):
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        
    def teach_pos(self, name):
        self.positions[name] = list(self.current_angles)
        self.log(f"Taught {name}: {[round(x,1) for x in self.positions[name]]}")
        messagebox.showinfo("Taught", f"{name} position saved successfully!")
        
    def manual_move(self, joint_idx, value):
        self.current_angles[joint_idx] = float(value)
        if self.connected:
            angles_str = "|".join(f"{a:.1f}" for a in self.current_angles)
            self.send_command(f"MOVE|{angles_str}")
            
    # --- Camera Logic ---
    def toggle_camera(self):
        if not self.camera_running:
            self.cap = cv2.VideoCapture(self.cam_index_var.get())
            if not self.cap.isOpened():
                messagebox.showerror("Error", "Could not open camera!")
                return
            self.camera_running = True
            self.cam_btn.config(text="Stop Camera")
            threading.Thread(target=self.camera_loop, daemon=True).start()
        else:
            self.camera_running = False
            self.cam_btn.config(text="Start Camera")
            if self.cap:
                self.cap.release()
            self.cam_label.config(image='', text="Camera Offline")
            self.color_label.config(text="DETECTED: NONE", foreground="black")
            
    def camera_loop(self):
        while self.camera_running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
                
            frame = cv2.flip(frame, 1)
            
            # Apply digital zoom
            zoom = self.zoom_var.get()
            if zoom > 1.0:
                h, w = frame.shape[:2]
                new_h, new_w = int(h / zoom), int(w / zoom)
                y1, y2 = (h - new_h) // 2, (h + new_h) // 2
                x1, x2 = (w - new_w) // 2, (w + new_w) // 2
                frame = frame[y1:y2, x1:x2]
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_LINEAR)
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Ranges for Red
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            # Ranges for Yellow
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            area_red = cv2.countNonZero(mask_red)
            area_yellow = cv2.countNonZero(mask_yellow)
            
            detected = "NONE"
            color = (255, 255, 255)
            
            if area_red > 1500 and area_red > area_yellow:
                detected = "RED"
                color = (0, 0, 255)
            elif area_yellow > 1500:
                detected = "YELLOW"
                color = (0, 255, 255)
                
            self.last_detected_color = detected
            
            # Safe UI Update
            self.root.after(0, self.update_cam_ui, frame, detected, color)
            time.sleep(0.05)
            
    def update_cam_ui(self, frame, detected, color):
        if not self.camera_running: return
        
        cv2.putText(frame, f"Color: {detected}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        
        w, h = 800, 600
        img = img.resize((w, h), Image.Resampling.LANCZOS)
        imgtk = ImageTk.PhotoImage(image=img)
        
        self.cam_label.imgtk = imgtk
        self.cam_label.configure(image=imgtk, text="")
        
        if detected == "RED":
            self.color_label.config(text="DETECTED: RED", foreground="red")
        elif detected == "YELLOW":
            self.color_label.config(text="DETECTED: YELLOW", foreground="orange")
        else:
            self.color_label.config(text="DETECTED: NONE", foreground="black")
            
    # --- Serial Logic ---
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_combo.set(ports[0])

    def toggle_connection(self):
        if not self.connected:
            port = self.port_var.get()
            if not port: return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(2)
                self.connected = True
                self.conn_btn.config(text="Disconnect Controller")
                threading.Thread(target=self.serial_reader, daemon=True).start()
                self.log("Connected to Controller.")
            except Exception as e:
                messagebox.showerror("Error", f"Connection Failed: {e}")
        else:
            self.connected = False
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
            self.conn_btn.config(text="Connect")
            self.log("Disconnected form Controller.")
            
    def serial_reader(self):
        buffer = ""
        while self.connected and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    buffer += self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if "MOVEMENT|COMPLETE" in line or "IDLE" in line:
                            self.movement_complete_event.set()
            except: break
            time.sleep(0.01)

    def send_command(self, cmd):
        if self.simulation_mode.get(): return True
        if self.serial_port and self.connected:
            try:
                self.serial_port.write((cmd + "\n").encode())
                return True
            except: return False
        return False
        
    def wait_for_ack(self, timeout=15.0):
        if self.simulation_mode.get(): time.sleep(0.5); return True
        self.movement_complete_event.clear()
        start = time.time()
        while (time.time() - start) < timeout:
            if not self.is_running or self.stop_requested: return False
            if self.movement_complete_event.is_set(): return True
            time.sleep(0.05)
        return False

    def execute_move(self, target_angles):
        if self.simulation_mode.get():
            time.sleep(1.0)
            self.current_angles = list(target_angles)
            for i in range(6): self.joint_sliders[i].set(self.current_angles[i])
            return True
            
        cmd = f"MOVE|{'|'.join(f'{a:.1f}' for a in target_angles)}"
        self.send_command(cmd)
        
        # Sync UI sliders
        self.current_angles = list(target_angles)
        for i in range(6): self.joint_sliders[i].set(self.current_angles[i])
            
        return self.wait_for_ack()

    def execute_setpos(self, target_angles):
        # SetPos ignores the base angle (J1), keeping the previous position's base angle
        target = list(target_angles)
        if len(target) == 5:
            target.insert(0, self.current_angles[0])
        elif len(target) >= 6:
            target[0] = self.current_angles[0]
            
        self.log("Executing SetPos (Base unchanged)...")
        
        if self.simulation_mode.get():
            time.sleep(1.0)
            self.current_angles = target
            for i in range(6): self.joint_sliders[i].set(self.current_angles[i])
            return True
            
        cmd = f"MOVE|{'|'.join(f'{a:.1f}' for a in target)}"
        if self.send_command(cmd):
            # Sync UI sliders
            self.current_angles = list(target)
            for i in range(6): self.joint_sliders[i].set(self.current_angles[i])
                
            time.sleep(0.1) # Sync delay
            return self.wait_for_ack(20.0)
        return False

    # --- Auto Cycle ---
    def start_cycle(self):
        if not self.connected and not self.simulation_mode.get():
            messagebox.showwarning("Warning", "Not connected to controller and not in Simulation Mode!")
            return
        if self.is_running: return
        self.is_running = True
        self.stop_requested = False
        self.run_btn.config(state='disabled')
        self.log("=== CYCLE STARTED ===")
        threading.Thread(target=self.cycle_loop, daemon=True).start()
        
    def stop_cycle(self):
        self.stop_requested = True
        self.is_running = False
        self.log("Cycle stop requested...")
        
    def emergency_stop(self):
        self.stop_requested = True
        self.is_running = False
        if self.connected: self.send_command("EMERGENCY_STOP")
        self.log("!!! EMERGENCY STOP !!!")

    def cycle_loop(self):
        try:
            self.log("Moving to Initial (Home)...")
            self.execute_move(self.positions["Home"])
            
            self.delay(1)
            
            self.log("Moving to SetPos...")
            self.execute_setpos(self.positions["SetPos"])
            
            self.delay(2)
            
            detected_color = self.last_detected_color
            self.log(f"Detected Color: {detected_color}")
            
            self.delay(3)
            
            self.log("Moving to PickStand...")
            self.execute_move(self.positions["PickStand"])
            
            self.delay(2)
            
            self.log("Moving to Pick Pos...")
            self.execute_move(self.positions["Pick"])
            
            self.delay(2)
            
            self.log("SetDO Activate (Gripper Open)...")
            self.send_command("GRIPPER|OPEN")
            
            self.delay(2)
            
            self.log("Moving to PickStand...")
            self.execute_move(self.positions["PickStand"])
            
            self.delay(2)
            
            if detected_color == "RED":
                self.log("Color RED -> Moving to RedPlaceStand")
                self.execute_move(self.positions["RedPlaceStand"])
                
                self.delay(2)
                
                self.log("Moving to RedPlace...")
                self.execute_move(self.positions["RedPlace"])
                
                self.delay(2)
                
                self.log("ResetDO (Gripper Close)...")
                self.send_command("GRIPPER|CLOSE")
                
                self.delay(2)
                
                self.log("Moving to RedPlaceStand...")
                self.execute_move(self.positions["RedPlaceStand"])
                
                self.delay(2)
                
                self.log("Moving to Home...")
                self.execute_move(self.positions["Home"])
                
            elif detected_color == "YELLOW":
                self.log("Color YELLOW -> Moving to YellowPlaceStand")
                self.execute_move(self.positions["YellowPlaceStand"])
                
                self.delay(2)
                
                self.log("Moving to YellowPlace...")
                self.execute_move(self.positions["YellowPlace"])
                
                self.delay(2)
                
                self.log("ResetDO (Gripper Close)...")
                self.send_command("GRIPPER|CLOSE")
                
                self.delay(2)
                
                self.log("Moving to YellowPlaceStand...")
                self.execute_move(self.positions["YellowPlaceStand"])
                
                self.delay(2)
                
                self.log("Moving to Home...")
                self.execute_move(self.positions["Home"])
                
            else:
                self.log("No valid color -> Staying in PickStand")
                self.delay(2)
                self.log("Skipping place routine due to unknown color.")
                self.log("Moving to Home...")
                self.execute_move(self.positions["Home"])
                
            self.log("Cycle Complete.")
            
        except Exception as e:
            if str(e) == "Stop requested":
                self.log("Cycle Interrupted.")
            else:
                self.log(f"Error in cycle: {e}")
        finally:
            self.is_running = False
            self.run_btn.config(state='normal')

    def delay(self, seconds):
        self.log(f"Delay {seconds}s...")
        start = time.time()
        while time.time() - start < seconds:
            if self.stop_requested:
                raise Exception("Stop requested")
            time.sleep(0.1)

if __name__ == "__main__":
    root = tk.Tk()
    app = AIProgrammer(root)
    root.mainloop()