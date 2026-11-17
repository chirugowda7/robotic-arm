import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time


# ============================================================
# SETTINGS
# ============================================================

BAUD_RATE = 115200

NUM_SERVOS = 5

SERVO_NAMES = [
    "Base (1)",
    "Shoulder (2)",
    "Elbow (3)",
    "Wrist Pitch (4)",
    "Wrist Roll (5)"
]

MIN_ANGLE = [
    0,
    30,
    60,
    30,
    0
]

MAX_ANGLE = [
    180,
    150,
    180,
    150,
    180
]


# ============================================================
# GLOBALS
# ============================================================

ser = None

running = True

current_values = [
    90,
    90,
    90,
    90,
    90
]


# ============================================================
# GUI
# ============================================================

root = tk.Tk()

root.title(
    "Industrial Robotic Arm Controller"
)

root.geometry(
    "900x650"
)

root.configure(
    bg="#eeeeee"
)


# ============================================================
# SERIAL FUNCTIONS
# ============================================================

def get_ports():

    ports = serial.tools.list_ports.comports()

    return [
        port.device
        for port in ports
    ]


def refresh_ports():

    ports = get_ports()

    port_combo["values"] = ports

    if ports:
        port_combo.current(0)


def connect_serial():

    global ser

    if ser is not None and ser.is_open:
        return

    port = port_combo.get()

    if not port:
        messagebox.showerror(
            "Connection",
            "Please select a COM port."
        )
        return

    try:

        ser = serial.Serial(
            port,
            BAUD_RATE,
            timeout=0.1
        )

        time.sleep(2)

        status_label.config(
            text="CONNECTED",
            foreground="green"
        )

        connect_button.config(
            text="Disconnect",
            command=disconnect_serial
        )

        # Start receiver thread
        threading.Thread(
            target=serial_reader,
            daemon=True
        ).start()

        send_command("PING")

    except Exception as e:

        messagebox.showerror(
            "Connection Error",
            str(e)
        )


def disconnect_serial():

    global ser

    try:

        if ser is not None:
            ser.close()

    except:
        pass

    ser = None

    status_label.config(
        text="DISCONNECTED",
        foreground="red"
    )

    connect_button.config(
        text="Connect",
        command=connect_serial
    )


def send_command(command):

    global ser

    if ser is None:
        return

    try:

        if ser.is_open:

            message = command + "\n"

            ser.write(
                message.encode("utf-8")
            )

    except Exception as e:

        print(
            "Serial error:",
            e
        )


# ============================================================
# SERIAL READER
# ============================================================

def serial_reader():

    global ser

    while running:

        try:

            if ser is not None and ser.is_open:

                line = ser.readline()

                if line:

                    text = line.decode(
                        "utf-8",
                        errors="ignore"
                    ).strip()

                    if text:

                        root.after(
                            0,
                            process_serial_message,
                            text
                        )

            else:

                time.sleep(0.1)

        except:

            time.sleep(0.1)


# ============================================================
# SERIAL MESSAGE PROCESSING
# ============================================================

def process_serial_message(message):

    print(
        "ESP32:",
        message
    )

    # --------------------------------------------------------
    # POSITION|90|90|90|90|90
    # --------------------------------------------------------

    if message.startswith("POSITION|"):

        parts = message.split("|")

        if len(parts) == 6:

            for i in range(NUM_SERVOS):

                try:

                    value = float(
                        parts[i + 1]
                    )

                    current_values[i] = value

                    sliders[i].set(value)

                    angle_labels[i].config(
                        text=f"{value:.1f}°"
                    )

                except:
                    pass

    # --------------------------------------------------------
    # STATUS
    # --------------------------------------------------------

    elif message.startswith("STATUS|"):

        parts = message.split("|")

        if len(parts) >= 2:

            state = parts[1]

            if state == "MOVING":

                system_status.config(
                    text="MOVING",
                    foreground="blue"
                )

            elif state == "IDLE":

                system_status.config(
                    text="IDLE",
                    foreground="green"
                )

            elif state == "ESTOP":

                system_status.config(
                    text="EMERGENCY STOP",
                    foreground="red"
                )

    # --------------------------------------------------------
    # READY
    # --------------------------------------------------------

    elif message == "ARM_READY":

        system_status.config(
            text="READY",
            foreground="green"
        )


# ============================================================
# SERVO CONTROL
# ============================================================

def slider_changed(index, value):

    angle = float(value)

    angle_labels[index].config(
        text=f"{angle:.1f}°"
    )


def set_servo(index):

    values = []

    for i in range(NUM_SERVOS):

        values.append(
            float(
                sliders[i].get()
            )
        )

    command = (
        "MOVE|" +
        "|".join(
            f"{value:.1f}"
            for value in values
        )
    )

    send_command(command)


def move_all():

    values = []

    for i in range(NUM_SERVOS):

        value = sliders[i].get()

        values.append(
            f"{float(value):.1f}"
        )

    command = (
        "MOVE|" +
        "|".join(values)
    )

    send_command(command)


def home():

    send_command(
        "HOME"
    )


# ============================================================
# EMERGENCY STOP
# ============================================================

def emergency_stop():

    send_command(
        "EMERGENCY_STOP"
    )

    system_status.config(
        text="EMERGENCY STOP",
        foreground="red"
    )


def reset_estop():

    send_command(
        "RESET_ESTOP"
    )

    system_status.config(
        text="IDLE",
        foreground="green"
    )


# ============================================================
# GRIPPER
# ============================================================

def gripper_open():

    send_command(
        "GRIPPER|OPEN"
    )


def gripper_close():

    send_command(
        "GRIPPER|CLOSE"
    )


# ============================================================
# SPEED
# ============================================================

def speed_changed(value):

    speed = float(value)

    speed_label.config(
        text=f"{speed:.0f}%"
    )


def set_speed():

    speed = speed_slider.get()

    send_command(
        f"SET_SPEED|{speed:.0f}"
    )


# ============================================================
# CONNECTION FRAME
# ============================================================

connection_frame = ttk.LabelFrame(
    root,
    text="Connection"
)

connection_frame.pack(
    fill="x",
    padx=15,
    pady=10
)


ttk.Label(
    connection_frame,
    text="Port:"
).grid(
    row=0,
    column=0,
    padx=8,
    pady=10
)


port_combo = ttk.Combobox(
    connection_frame,
    width=12,
    state="readonly"
)

port_combo.grid(
    row=0,
    column=1,
    padx=5
)


refresh_button = ttk.Button(
    connection_frame,
    text="Refresh",
    command=refresh_ports
)

refresh_button.grid(
    row=0,
    column=2,
    padx=5
)


ttk.Label(
    connection_frame,
    text="Baud:"
).grid(
    row=0,
    column=3,
    padx=8
)


ttk.Label(
    connection_frame,
    text="115200"
).grid(
    row=0,
    column=4,
    padx=5
)


connect_button = ttk.Button(
    connection_frame,
    text="Connect",
    command=connect_serial
)

connect_button.grid(
    row=0,
    column=5,
    padx=10
)


status_label = tk.Label(
    connection_frame,
    text="DISCONNECTED",
    fg="red",
    bg="#eeeeee",
    font=("Arial", 10, "bold")
)

status_label.grid(
    row=0,
    column=6,
    padx=10
)


# ============================================================
# SERVO CONTROL FRAME
# ============================================================

servo_frame = ttk.LabelFrame(
    root,
    text="Servo Control"
)

servo_frame.pack(
    fill="both",
    expand=True,
    padx=15,
    pady=10
)


sliders = []

angle_labels = []


for i in range(NUM_SERVOS):

    row = i

    # --------------------------------------------------------
    # Servo name
    # --------------------------------------------------------

    ttk.Label(
        servo_frame,
        text=SERVO_NAMES[i],
        width=16
    ).grid(
        row=row,
        column=0,
        padx=10,
        pady=12,
        sticky="w"
    )

    # --------------------------------------------------------
    # Slider
    # --------------------------------------------------------

    slider = tk.Scale(
        servo_frame,
        from_=MIN_ANGLE[i],
        to=MAX_ANGLE[i],
        orient="horizontal",
        length=430,
        resolution=0.1,
        showvalue=False,
        command=lambda value, index=i:
            slider_changed(index, value)
    )

    slider.set(90)

    slider.grid(
        row=row,
        column=1,
        padx=5,
        pady=10
    )

    sliders.append(slider)

    # --------------------------------------------------------
    # Angle
    # --------------------------------------------------------

    label = ttk.Label(
        servo_frame,
        text="90.0°",
        width=8
    )

    label.grid(
        row=row,
        column=2,
        padx=10
    )

    angle_labels.append(label)

    # --------------------------------------------------------
    # Set button
    # --------------------------------------------------------

    button = ttk.Button(
        servo_frame,
        text="Set",
        command=lambda index=i:
            set_servo(index)
    )

    button.grid(
        row=row,
        column=3,
        padx=10
    )


# ============================================================
# SPEED
# ============================================================

speed_frame = ttk.LabelFrame(
    root,
    text="Movement Speed"
)

speed_frame.pack(
    fill="x",
    padx=15,
    pady=5
)


speed_slider = tk.Scale(
    speed_frame,
    from_=10,
    to=300,
    orient="horizontal",
    length=500,
    showvalue=False,
    command=speed_changed
)

speed_slider.set(100)

speed_slider.pack(
    side="left",
    padx=15
)


speed_label = ttk.Label(
    speed_frame,
    text="100%"
)

speed_label.pack(
    side="left",
    padx=10
)


speed_button = ttk.Button(
    speed_frame,
    text="Set Speed",
    command=set_speed
)

speed_button.pack(
    side="left",
    padx=10
)


# ============================================================
# CONTROL BUTTONS
# ============================================================

control_frame = tk.Frame(
    root,
    bg="#eeeeee"
)

control_frame.pack(
    fill="x",
    padx=15,
    pady=10
)


home_button = ttk.Button(
    control_frame,
    text="HOME",
    command=home
)

home_button.pack(
    side="left",
    padx=5
)


move_button = ttk.Button(
    control_frame,
    text="MOVE ALL",
    command=move_all
)

move_button.pack(
    side="left",
    padx=5
)


open_button = ttk.Button(
    control_frame,
    text="GRIPPER OPEN",
    command=gripper_open
)

open_button.pack(
    side="left",
    padx=5
)


close_button = ttk.Button(
    control_frame,
    text="GRIPPER CLOSE",
    command=gripper_close
)

close_button.pack(
    side="left",
    padx=5
)


stop_button = tk.Button(
    control_frame,
    text="EMERGENCY STOP",
    bg="red",
    fg="white",
    font=("Arial", 10, "bold"),
    command=emergency_stop
)

stop_button.pack(
    side="right",
    padx=5
)


reset_button = tk.Button(
    control_frame,
    text="RESET E-STOP",
    bg="green",
    fg="white",
    font=("Arial", 10, "bold"),
    command=reset_estop
)

reset_button.pack(
    side="right",
    padx=5
)


# ============================================================
# SYSTEM STATUS
# ============================================================

system_frame = ttk.LabelFrame(
    root,
    text="System Status"
)

system_frame.pack(
    fill="x",
    padx=15,
    pady=5
)


system_status = tk.Label(
    system_frame,
    text="DISCONNECTED",
    fg="red",
    bg="#eeeeee",
    font=("Arial", 11, "bold")
)

system_status.pack(
    pady=8
)


# ============================================================
# STARTUP
# ============================================================

refresh_ports()


# ============================================================
# CLOSE WINDOW
# ============================================================

def close_program():

    global running

    running = False

    disconnect_serial()

    root.destroy()


root.protocol(
    "WM_DELETE_WINDOW",
    close_program
)


# ============================================================
# RUN
# ============================================================

root.mainloop()