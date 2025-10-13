import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import time
import threading
import logging
import numpy as np
from typing import Optional, Callable

# 1 mm = 1000 microns = 10000 tenths of a micron.
UNITS_MM_TO_DEVICE = 10000

class MS2000Controller:
    """
    Handles all communication and control logic for the ASI MS-2000 stage.
    """
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None
        self.port = "COM4"
        self.baudrate = 9600
        self.timeout = 1.0
        self.is_running_scan = False
        self.stop_event = threading.Event()
        self.log = log_callback
        self.lock = threading.Lock() # Ensures one command at a time

    def connect(self, port: str, baudrate: int) -> bool:
        """Establishes a connection with the controller."""
        if self.is_connected():
            self.log("Already connected.")
            return True
        try:
            self.port = port
            self.baudrate = baudrate
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(0.1) # Allow port to initialize
            self._set_high_precision()
            self.log(f"Successfully connected to MS-2000 on {self.port}.")
            return True
        except serial.SerialException as e:
            self.log(f"ERROR: Failed to connect to {self.port}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        """Closes the serial connection."""
        if self.is_running_scan:
            self.stop_scan()
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("Disconnected from MS-2000.")
        self.ser = None

    def is_connected(self) -> bool:
        """Returns True if the serial port is connected and open."""
        return self.ser is not None and self.ser.is_open

    def _flush_buffers(self):
        """Clears serial input and output buffers."""
        if self.ser:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def _set_high_precision(self):
        """
        Switches the controller to high-precision mode for WHERE commands.
        Sends command bytes 255 72 (Alt+255, H).
        """
        if not self.ser: return
        try:
            cmd = bytes([255, 72])
            self.log("Switching to high-precision mode.")
            self._flush_buffers()
            self.ser.write(cmd)
            time.sleep(0.1) # This command has no standard reply
        except Exception as e:
            self.log(f"ERROR: Could not set high-precision mode: {e}")

    def send_command(self, command: str) -> Optional[str]:
        """
        Sends a command to the controller and returns the response.
        This method is thread-safe.
        """
        if not self.is_connected():
            self.log("ERROR: Not connected.")
            return None

        with self.lock:
            try:
                self.log(f"CMD > {command}")
                self._flush_buffers()
                self.ser.write(f"{command}\r".encode('ascii'))
                response = self.ser.read_until(b'\r\n').decode('ascii').strip()
                self.log(f"RSP < {response}")

                if response.startswith(":N"):
                    error_code = response.split()[0]
                    self.log(f"WARNING: Controller returned error: {error_code}")

                return response
            except Exception as e:
                self.log(f"ERROR: Failed to send command '{command}': {e}")
                return None

    def get_position(self) -> Optional[tuple[float, float]]:
        """Queries the current X and Y stage position in mm."""
        response = self.send_command("W X Y")
        if response and response.startswith(":A"):
            try:
                parts = response.split()
                # Response is in tenths of microns, convert to mm
                x_mm = float(parts[1]) / UNITS_MM_TO_DEVICE
                y_mm = float(parts[2]) / UNITS_MM_TO_DEVICE
                return x_mm, y_mm
            except (ValueError, IndexError) as e:
                self.log(f"ERROR: Could not parse position response: {response} ({e})")
                return None
        return None

    def get_status(self) -> str:
        """Checks if the stage is busy ('B') or not busy ('N')."""
        response = self.send_command("/")
        if response:
            return response
        return 'E' # Error

    def wait_for_idle(self):
        """Polls the status and waits until the stage is no longer busy."""
        self.log("Waiting for move to complete...")
        while not self.stop_event.is_set():
            status = self.get_status()
            if status == 'N':
                self.log("Move complete.")
                return
            if status == 'E':
                self.log("ERROR: Could not get status while waiting.")
                return
            time.sleep(0.05) # Poll every 50ms

    def move_absolute(self, x_mm: float, y_mm: float):
        """Moves the stage to absolute coordinates specified in mm."""
        x_dev = int(x_mm * UNITS_MM_TO_DEVICE)
        y_dev = int(y_mm * UNITS_MM_TO_DEVICE)
        self.send_command(f"M X={x_dev} Y={y_dev}")

    def move_relative(self, dx_mm: float, dy_mm: float):
        """Moves the stage by a relative amount specified in mm."""
        dx_dev = int(dx_mm * UNITS_MM_TO_DEVICE)
        dy_dev = int(dy_mm * UNITS_MM_TO_DEVICE)
        self.send_command(f"R X={dx_dev} Y={dy_dev}")

    def set_speed(self, speed_mms: float):
        """Sets the maximum speed for both axes in mm/s."""
        self.send_command(f"S X={speed_mms} Y={speed_mms}")

    def zero_position(self):
        """Sets the current position as the origin (0, 0)."""
        self.send_command("Z")

    def halt(self):
        """Immediately stops all stage movement."""
        # Use '\' which is ASCII 92
        self.send_command(chr(92))

    def run_scan(self, params: dict):
        """Executes a raster scan based on the given parameters."""
        self.is_running_scan = True
        self.stop_event.clear()
        self.log("--- Starting Scan ---")

        try:
            # Generate scan points
            x_points = np.linspace(params['start_x'], params['end_x'], params['steps_x'])
            y_points = np.linspace(params['start_y'], params['end_y'], params['steps_y'])

            self.set_speed(params['speed'])
            total_points = len(x_points) * len(y_points)
            point_count = 0

            # Go to the starting position
            self.log(f"Moving to start position ({x_points[0]:.4f}, {y_points[0]:.4f})...")
            self.move_absolute(x_points[0], y_points[0])
            self.wait_for_idle()

            if self.stop_event.is_set(): raise InterruptedError

            # Main scan loop
            for i, y in enumerate(y_points):
                # For serpentine scans, reverse direction on even-numbered rows
                current_x_points = x_points if i % 2 == 0 else x_points[::-1]

                for x in current_x_points:
                    if self.stop_event.is_set(): raise InterruptedError

                    point_count += 1
                    self.log(f"Moving to point {point_count}/{total_points}: ({x:.4f}, {y:.4f})")

                    self.move_absolute(x, y)
                    self.wait_for_idle()

                    self.log(f"Dwelling for {params['dwell']}s...")
                    time.sleep(params['dwell']) # Simulate data acquisition
            
            self.log("--- Scan Completed Successfully ---")

        except InterruptedError:
            self.log("--- Scan Stopped by User ---")
            self.halt()
        except Exception as e:
            self.log(f"--- ERROR during scan: {e} ---")
            self.halt()
        finally:
            self.is_running_scan = False

    def stop_scan(self):
        """Signals the scanning thread to stop."""
        self.log("Stop signal received.")
        self.stop_event.set()


class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Stage Scan Controller")
        self.root.geometry("800x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- Controller ---
        self.controller = MS2000Controller(self.log_message)
        self.scan_thread: Optional[threading.Thread] = None

        # --- GUI Setup ---
        self._create_widgets()
        self.update_status_loop()

    def _create_widgets(self):
        # Main frames
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # --- Left Frame Widgets ---
        conn_frame = ttk.LabelFrame(left_frame, text="Connection", padding="10")
        conn_frame.pack(fill=tk.X, pady=5)

        status_frame = ttk.LabelFrame(left_frame, text="Live Status", padding="10")
        status_frame.pack(fill=tk.X, pady=5)

        manual_frame = ttk.LabelFrame(left_frame, text="Manual Control", padding="10")
        manual_frame.pack(fill=tk.X, pady=5)
        
        scan_frame = ttk.LabelFrame(left_frame, text="Scan Control", padding="10")
        scan_frame.pack(fill=tk.X, pady=5)

        # --- Right Frame Widgets ---
        log_frame = ttk.LabelFrame(right_frame, text="Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True)

        # == Connection Frame ==
        self.entries = {}
        conn_fields = {"Port": "COM4", "Baudrate": "9600"}
        for i, (text, val) in enumerate(conn_fields.items()):
            ttk.Label(conn_frame, text=f"{text}:").grid(row=i, column=0, sticky="w", padx=5, pady=2)
            entry = ttk.Entry(conn_frame, width=12)
            entry.insert(0, val)
            entry.grid(row=i, column=1, sticky="ew", padx=5, pady=2)
            self.entries[text.lower()] = entry
        
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=2, column=0, columnspan=2, pady=5, sticky="ew")
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_button.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew")

        # == Status Frame ==
        self.pos_labels = {}
        for i, axis in enumerate(["X", "Y"]):
            ttk.Label(status_frame, text=f"{axis}:", font=("TkDefaultFont", 10, "bold")).grid(row=i, column=0, sticky="w")
            label = ttk.Label(status_frame, text="0.0000 mm", font=("Courier", 10), width=15)
            label.grid(row=i, column=1, sticky="e")
            self.pos_labels[axis] = label

        # == Manual Control Frame ==
        # Relative Move
        rel_move_frame = ttk.Frame(manual_frame)
        rel_move_frame.pack(pady=5)
        ttk.Label(rel_move_frame, text="Step (mm):").pack(side=tk.LEFT, padx=5)
        self.step_entry = ttk.Entry(rel_move_frame, width=6)
        self.step_entry.insert(0, "0.1")
        self.step_entry.pack(side=tk.LEFT)

        buttons_frame = ttk.Frame(manual_frame)
        buttons_frame.pack(pady=5)
        ttk.Button(buttons_frame, text="←", command=lambda: self.move_rel(-1, 0)).grid(row=1, column=0)
        ttk.Button(buttons_frame, text="→", command=lambda: self.move_rel(1, 0)).grid(row=1, column=2)
        ttk.Button(buttons_frame, text="↑", command=lambda: self.move_rel(0, 1)).grid(row=0, column=1)
        ttk.Button(buttons_frame, text="↓", command=lambda: self.move_rel(0, -1)).grid(row=2, column=1)

        # Absolute Move
        abs_move_frame = ttk.Frame(manual_frame)
        abs_move_frame.pack(pady=10)
        ttk.Label(abs_move_frame, text="Go To X:").grid(row=0, column=0, pady=2)
        self.goto_x_entry = ttk.Entry(abs_move_frame, width=8)
        self.goto_x_entry.insert(0, "0")
        self.goto_x_entry.grid(row=0, column=1)
        ttk.Label(abs_move_frame, text="Y:").grid(row=0, column=2)
        self.goto_y_entry = ttk.Entry(abs_move_frame, width=8)
        self.goto_y_entry.insert(0, "0")
        self.goto_y_entry.grid(row=0, column=3)
        ttk.Button(abs_move_frame, text="Go", command=self.go_to_abs).grid(row=0, column=4, padx=5)
        
        ttk.Button(manual_frame, text="Set Current as (0, 0)", command=self.zero_stage).pack(pady=5, fill=tk.X)
        
        # == Scan Frame (Parameters) ==
        scan_param_frame = ttk.LabelFrame(left_frame, text="Scan Parameters", padding="10")
        scan_param_frame.pack(fill=tk.X, pady=5)
        self.scan_entries = {}
        scan_fields = {
            "Start X (mm)": "0", "End X (mm)": "1", "Steps X": "10",
            "Start Y (mm)": "0", "End Y (mm)": "1", "Steps Y": "10",
            "Speed (mm/s)": "1.0", "Dwell (s)": "0.1"
        }
        for i, (text, val) in enumerate(scan_fields.items()):
            key = text.lower().replace(" (mm)", "").replace(" (s)", "").replace(" ", "_")
            ttk.Label(scan_param_frame, text=f"{text}:").grid(row=i, column=0, sticky="w", pady=2)
            entry = ttk.Entry(scan_param_frame, width=10)
            entry.insert(0, val)
            entry.grid(row=i, column=1, sticky="ew")
            self.scan_entries[key] = entry
        
        # == Scan Control Buttons ==
        self.start_scan_button = ttk.Button(scan_frame, text="Start Scan", command=self.start_scan, state=tk.DISABLED)
        self.start_scan_button.pack(fill=tk.X, pady=5)
        self.stop_scan_button = ttk.Button(scan_frame, text="Stop Scan", command=self.stop_scan, state=tk.DISABLED)
        self.stop_scan_button.pack(fill=tk.X)
        
        # == Log Area ==
        self.log_area = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=10)
        self.log_area.pack(fill=tk.BOTH, expand=True)
        self.log_area.configure(state='disabled')

    # --- GUI Actions ---
    def connect(self):
        port = self.entries['port'].get()
        baud = int(self.entries['baudrate'].get())
        if self.controller.connect(port, baud):
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.start_scan_button.config(state=tk.NORMAL)

    def disconnect(self):
        self.controller.disconnect()
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.start_scan_button.config(state=tk.DISABLED)
        self.stop_scan_button.config(state=tk.DISABLED)

    def move_rel(self, dx_factor, dy_factor):
        try:
            step = float(self.step_entry.get())
            self.controller.move_relative(step * dx_factor, step * dy_factor)
        except ValueError:
            self.log_message("ERROR: Invalid step value.")
            
    def go_to_abs(self):
        try:
            x = float(self.goto_x_entry.get())
            y = float(self.goto_y_entry.get())
            self.controller.move_absolute(x, y)
        except ValueError:
            self.log_message("ERROR: Invalid Go To coordinates.")
            
    def zero_stage(self):
        self.controller.zero_position()

    def get_scan_params(self) -> Optional[dict]:
        """Reads and validates scan parameters from the GUI."""
        params = {}
        try:
            for key, entry in self.scan_entries.items():
                if key in ['steps_x', 'steps_y']:
                    params[key] = int(entry.get())
                else:
                    params[key] = float(entry.get())
            
            if params['steps_x'] < 2 or params['steps_y'] < 2:
                self.log_message("ERROR: Steps for X and Y must be 2 or greater.")
                return None
            return params
        except ValueError:
            self.log_message("ERROR: Invalid scan parameter. Please enter numbers only.")
            return None

    def start_scan(self):
        if self.controller.is_running_scan:
            self.log_message("Scan is already running.")
            return
            
        params = self.get_scan_params()
        if params is None: return

        self.start_scan_button.config(state=tk.DISABLED)
        self.stop_scan_button.config(state=tk.NORMAL)

        self.scan_thread = threading.Thread(
            target=self.controller.run_scan, args=(params,), daemon=True
        )
        self.scan_thread.start()
        self.check_scan_thread()

    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive():
            self.controller.stop_scan()
        # GUI will be re-enabled by check_scan_thread

    def check_scan_thread(self):
        """Checks if the scan thread is alive and updates GUI state."""
        if self.scan_thread and self.scan_thread.is_alive():
            self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected():
                self.start_scan_button.config(state=tk.NORMAL)

    def update_status_loop(self):
        """Periodically updates the position display."""
        if self.controller.is_connected():
            position = self.controller.get_position()
            if position:
                x, y = position
                self.pos_labels["X"].config(text=f"{x:9.4f} mm")
                self.pos_labels["Y"].config(text=f"{y:9.4f} mm")
        else:
            self.pos_labels["X"].config(text="--.---- mm")
            self.pos_labels["Y"].config(text="--.---- mm")
            
        self.root.after(500, self.update_status_loop) # Update every 500ms

    def log_message(self, message: str):
        """Thread-safe method to append a message to the log area."""
        def append():
            self.log_area.configure(state='normal')
            self.log_area.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
            self.log_area.configure(state='disabled')
            self.log_area.see(tk.END)
        self.root.after(0, append)

    def on_closing(self):
        """Handles the window closing event."""
        self.log_message("Closing application...")
        self.controller.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()
    
# PS C:\Users\QE LAB> [System.IO.Ports.SerialPort]::new(
# >>     'COM4',
# >>     9600,
# >>     [System.IO.Ports.Parity]::None,
# >>     8,
# >>     [System.IO.Ports.StopBits]::One
# >> ).{
# >>     $_.Open()
# >>     $_.WriteLine('PING')
# >>     Start-Sleep -Milliseconds 300
# >>     $out = $_.ReadExisting()
# >>     $_.Close()
# >>     $out   # ?????? ?????????? ??????????
# >> }
# >>
# PS C:\Users\QE LAB>
# 13:04:43 - Switching to high-precision mode.
# 13:04:43 - Successfully connected to MS-2000 on COM4.
# 13:04:43 - CMD > W X Y
# 13:04:43 - RSP < :A 8000.0 -0.4
# 13:04:43 - CMD > W X Y
# 13:04:43 - RSP < :A 8000.0 -0.4
# 13:04:44 - CMD > W X Y
# 13:04:44 - RSP < :A 8000.0 -0.4
# 13:04:44 - CMD > W X Y
# 13:04:44 - RSP < :A 8000.0 -0.4
# 13:04:45 - CMD > W X Y
# 13:04:45 - RSP < :A 8000.0 -0.4
# 13:04:45 - --- Starting Scan ---
# 13:04:45 - --- ERROR during scan: 'speed' ---
# 13:04:45 - CMD > \
# 13:04:45 - RSP < A:A
# 13:04:45 - CMD > W X Y
# 13:04:45 - RSP < :A 8000.0 -0.4
# 13:04:46 - CMD > W X Y
# 13:04:46 - RSP < :A 8000.0 -0.4
# 13:04:47 - CMD > W X Y
# 13:04:47 - RSP < :A 8000.0 -0.4
# 13:04:47 - CMD > W X Y
# 13:04:47 - RSP < :A 8000.0 -0.4
# 13:04:48 - CMD > W X Y
