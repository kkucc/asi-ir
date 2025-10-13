import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import time
import threading
import numpy as np
from typing import Optional, Callable

# 1 mm = 1000 microns = 10000 tenths of a micron.
UNITS_MM_TO_DEVICE = 10000

class MS2000Controller:
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None
        self.is_running_scan = False
        self.stop_event = threading.Event()
        self.log = log_callback
        self.lock = threading.Lock()

    def connect(self, port: str, baudrate: int) -> bool:
        if self.is_connected():
            self.log("Already connected.")
            return True
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            # FIX: Добавляем паузу, чтобы порт и устройство успели инициализироваться
            time.sleep(0.2)
            self._set_high_precision()
            self.log(f"Successfully connected to MS-2000 on {port}.")
            return True
        except serial.SerialException as e:
            self.log(f"ERROR: Failed to connect to {port}: {e}")
            self.ser = None
            return False

    def disconnect(self):
        if self.is_running_scan: self.stop_scan()
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("Disconnected from MS-2000.")
        self.ser = None

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _set_high_precision(self):
        if not self.ser: return
        try:
            cmd = bytes([255, 72])
            self.log("CMD > Switching to high-precision mode.")
            if self.ser:
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
            time.sleep(0.1)
        except Exception as e:
            self.log(f"ERROR: Could not set high-precision mode: {e}")

    def send_command(self, command: str) -> Optional[str]:
        if not self.is_connected():
            self.log("ERROR: Not connected.")
            return None
        with self.lock:
            try:
                self.log(f"CMD > {command}")
                if self.ser:
                    self.ser.reset_input_buffer()
                    self.ser.write(f"{command}\r".encode('ascii'))
                    response = self.ser.read_until(b'\r\n').decode('ascii').strip()
                    self.log(f"RSP < {response}")
                    if response.startswith(":N"):
                        self.log(f"WARNING: Controller returned error: {response}")
                    return response
            except Exception as e:
                self.log(f"ERROR: Failed to send command '{command}': {e}")
            return None

    def wait_for_idle(self):
        self.log("Waiting for move to complete...")
        while not self.stop_event.is_set():
            status = self.send_command("/")
            if status == 'N':
                self.log("Move complete.")
                return
            if status is None or status == 'E':
                self.log("ERROR: Could not get status while waiting.")
                raise ConnectionError("Lost connection or failed to get status.")
            time.sleep(0.05)

    def move_absolute(self, x_mm: float, y_mm: float):
        x_dev = int(x_mm * UNITS_MM_TO_DEVICE)
        y_dev = int(y_mm * UNITS_MM_TO_DEVICE)
        self.send_command(f"M X={x_dev} Y={y_dev}")

    def set_speed(self, speed_mms: float):
        self.send_command(f"S X={speed_mms} Y={speed_mms}")

    def halt(self):
        self.send_command(chr(92))

    def run_scan_step_and_measure(self, params: dict, line_callback: Callable):
        self.is_running_scan = True
        self.stop_event.clear()
        self.log("--- Starting Scan (Step-and-Measure Mode) ---")
        try:
            start_x, start_y = params['start_x'], params['start_y']
            steps_x, steps_y = int(params['steps_x']), int(params['steps_y'])
            step_x, step_y = params['step_x'], params['step_y']
            
            x_coords = np.linspace(start_x, start_x + step_x * (steps_x - 1), steps_x)
            y_coords = np.linspace(start_y, start_y + step_y * (steps_y - 1), steps_y)
            results = np.full((steps_y, steps_x), np.nan)

            self.set_speed(params['speed'])
            
            for i, y in enumerate(y_coords):
                current_x_coords = x_coords if i % 2 == 0 else x_coords[::-1]
                for j, x in enumerate(current_x_coords):
                    if self.stop_event.is_set(): raise InterruptedError
                    self.log(f"Moving to ({x:.4f}, {y:.4f})...")
                    self.move_absolute(x, y)
                    self.wait_for_idle()

                    self.log(f"Dwelling for {params['dwell']}s...")
                    time.sleep(params['dwell']) # ЗАГЛУШКА: Здесь будет сбор данных
                    
                    # Имитация полученного сигнала
                    value = np.random.rand() * 100
                    col_idx = j if i % 2 == 0 else (steps_x - 1 - j)
                    results[i, col_idx] = value
                
                if line_callback: line_callback(results.copy())

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
        self.log("Stop signal received.")
        self.stop_event.set()

class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Scan Platform v2")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.controller = MS2000Controller(self.log_message)
        self.scan_thread: Optional[threading.Thread] = None
        self._create_widgets()
        self.update_calculated_fields()

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        # ... (Frames setup) ...
        left_frame = ttk.Frame(main_frame); left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        right_frame = ttk.Frame(main_frame); right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        conn_frame = ttk.LabelFrame(left_frame, text="Connection", padding="10"); conn_frame.pack(fill=tk.X, pady=5)
        scan_param_frame = ttk.LabelFrame(left_frame, text="Scan Parameters", padding="10"); scan_param_frame.pack(fill=tk.X, pady=5)
        scan_control_frame = ttk.LabelFrame(left_frame, text="Scan Control", padding="10"); scan_control_frame.pack(fill=tk.X, pady=5)
        log_frame = ttk.LabelFrame(right_frame, text="Log", padding="10"); log_frame.pack(fill=tk.BOTH, expand=True)

        self.conn_entries = {}
        # ... (Connection widgets setup) ...
        for i, (text, val) in enumerate({"Port": "COM4", "Baudrate": "9600"}.items()):
            ttk.Label(conn_frame, text=f"{text}:").grid(row=i, column=0, sticky="w"); entry = ttk.Entry(conn_frame, width=12); entry.insert(0, val); entry.grid(row=i, column=1, sticky="ew"); self.conn_entries[text.lower()] = entry
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect); self.connect_button.grid(row=2, column=0, columnspan=2, pady=5, sticky="ew")
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED); self.disconnect_button.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew")

        self.scan_entries = {}
        scan_fields = {
            "start_x": ("Start X (mm)", "0.0"), "start_y": ("Start Y (mm)", "0.0"),
            "steps_x": ("Points X", "50"),      "steps_y": ("Points Y", "50"),
            "step_x":  ("Step X (mm)", "0.1"),   "step_y":  ("Step Y (mm)", "0.1"),
            "speed":   ("Speed (mm/s)", "1.0"), "dwell":   ("Dwell (s)", "0.01"),
        }
        
        i = 0
        for key, (label, value) in scan_fields.items():
            ttk.Label(scan_param_frame, text=label + ":").grid(row=i, column=0, sticky="w", pady=2)
            entry = ttk.Entry(scan_param_frame, width=10)
            entry.insert(0, value)
            entry.grid(row=i, column=1, sticky="ew", padx=5)
            # Привязываем событие изменения поля к функции обновления
            entry.bind("<KeyRelease>", self.update_calculated_fields)
            self.scan_entries[key] = entry
            i += 1
            
        self.calc_labels = {}
        calc_fields = {"End X": "end_x", "End Y": "end_y", "Area": "area"}
        for label, key in calc_fields.items():
            ttk.Label(scan_param_frame, text=label + ":").grid(row=i, column=0, sticky="w", pady=2)
            lbl = ttk.Label(scan_param_frame, text="0.0 mm", relief="sunken", width=10)
            lbl.grid(row=i, column=1, sticky="ew", padx=5)
            self.calc_labels[key] = lbl
            i += 1

        # Scan Control 
        self.start_scan_button = ttk.Button(scan_control_frame, text="Start Scan", command=self.start_scan, state=tk.DISABLED); self.start_scan_button.pack(fill=tk.X, pady=5)
        self.stop_scan_button = ttk.Button(scan_control_frame, text="Stop Scan", command=self.stop_scan, state=tk.DISABLED); self.stop_scan_button.pack(fill=tk.X)
        
        self.log_area = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=10); self.log_area.pack(fill=tk.BOTH, expand=True); self.log_area.configure(state='disabled')

    def update_calculated_fields(self, event=None):
        try:
            start_x = float(self.scan_entries["start_x"].get())
            steps_x = int(self.scan_entries["steps_x"].get())
            step_x = float(self.scan_entries["step_x"].get())
            
            start_y = float(self.scan_entries["start_y"].get())
            steps_y = int(self.scan_entries["steps_y"].get())
            step_y = float(self.scan_entries["step_y"].get())

            if steps_x < 1 or steps_y < 1: raise ValueError("Steps must be >= 1")

            end_x = start_x + (steps_x - 1) * step_x
            end_y = start_y + (steps_y - 1) * step_y
            area_x = abs((steps_x -1) * step_x) if steps_x > 1 else 0
            area_y = abs((steps_y -1) * step_y) if steps_y > 1 else 0

            self.calc_labels["end_x"].config(text=f"{end_x:.4f} mm")
            self.calc_labels["end_y"].config(text=f"{end_y:.4f} mm")
            self.calc_labels["area"].config(text=f"{area_x:.2f} x {area_y:.2f} mm")
        except (ValueError, TclError):
            # Если в поле ввода некорректное значение, просто ничего не делаем
            self.calc_labels["end_x"].config(text="Error")
            self.calc_labels["end_y"].config(text="Error")
            self.calc_labels["area"].config(text="Error")

    def get_scan_params(self) -> Optional[dict]:
        params = {}
        try:
            for key, entry in self.scan_entries.items():
                params[key] = float(entry.get())
            # Проверяем, что количество шагов - целые числа
            if params['steps_x'] % 1 != 0 or params['steps_y'] % 1 != 0:
                self.log_message("ERROR: Points X/Y must be integers.")
                return None
            if params['steps_x'] < 1 or params['steps_y'] < 1:
                self.log_message("ERROR: Points must be 1 or greater.")
                return None
            return params
        except (ValueError, TclError):
            self.log_message("ERROR: Invalid scan parameter. Please enter numbers only.")
            return None

    def start_scan(self):
        if self.controller.is_running_scan:
            self.log_message("Scan is already running."); return
        params = self.get_scan_params()
        if params is None: return

        self.start_scan_button.config(state=tk.DISABLED)
        self.stop_scan_button.config(state=tk.NORMAL)
        
        # ЗАГЛУШКА ДЛЯ line_callback - в будущих версиях здесь будет обновление графика
        def dummy_line_callback(data_line):
            print(f"Received line {data_line.shape[0]}")

        self.scan_thread = threading.Thread(
            target=self.controller.run_scan_step_and_measure,
            args=(params, dummy_line_callback), # Передаем callback
            daemon=True
        )
        self.scan_thread.start()
        self.check_scan_thread()
    
    # ... (Остальные методы: connect, disconnect, stop_scan, check_scan_thread, log_message, on_closing) ...
    def connect(self):
        port = self.conn_entries['port'].get(); baud = int(self.conn_entries['baudrate'].get())
        if self.controller.connect(port, baud):
            self.connect_button.config(state=tk.DISABLED); self.disconnect_button.config(state=tk.NORMAL); self.start_scan_button.config(state=tk.NORMAL)
    def disconnect(self):
        self.controller.disconnect(); self.connect_button.config(state=tk.NORMAL); self.disconnect_button.config(state=tk.DISABLED); self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.DISABLED)
    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.controller.stop_scan()
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected(): self.start_scan_button.config(state=tk.NORMAL)
    def on_closing(self): self.log_message("Closing..."); self.controller.disconnect(); self.root.destroy()
    def log_message(self, message: str):
        def append(): self.log_area.configure(state='normal'); self.log_area.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n"); self.log_area.configure(state='disabled'); self.log_area.see(tk.END)
        self.root.after(0, append)


if __name__ == "__main__":
    from tkinter import TclError
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()