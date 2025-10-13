import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import time
import threading
import numpy as np
from typing import Optional, Callable

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

# 1 mm = 1000 microns = 10000 tenths of a micron.
UNITS_MM_TO_DEVICE = 10000
FIELD_OF_VIEW_SIZE = 100 

class MS2000Controller:
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None; self.is_running_scan = False
        self.stop_event = threading.Event(); self.log = log_callback
        self.lock = threading.Lock()

    def connect(self, port: str, baudrate: int) -> bool:
        if self.is_connected(): return True
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(0.2); self._set_high_precision()
            self.log(f"Successfully connected to MS-2000 on {port}.")
            return True
        except serial.SerialException as e:
            self.log(f"ERROR: Failed to connect: {e}"); self.ser = None; return False

    def disconnect(self):
        if self.is_running_scan: self.stop_scan()
        if self.ser and self.ser.is_open: self.ser.close(); self.log("Disconnected.")
        self.ser = None
        
    def is_connected(self) -> bool: return self.ser is not None and self.ser.is_open

    def _set_high_precision(self):
        if not self.ser: return
        try:
            self.log("CMD > Switching to high-precision mode.")
            self.ser.reset_input_buffer(); self.ser.write(bytes([255, 72])); time.sleep(0.1)
        except Exception as e: self.log(f"ERROR: Could not set high-precision mode: {e}")

    def send_command(self, command: str) -> Optional[str]:
        if not self.is_connected(): return None
        with self.lock:
            try:
                self.log(f"CMD > {command}")
                self.ser.reset_input_buffer(); self.ser.write(f"{command}\r".encode('ascii'))
                response = self.ser.read_until(b'\r\n').decode('ascii').strip()
                self.log(f"RSP < {response}")
                return response
            except Exception as e: self.log(f"ERROR: Failed to send command '{command}': {e}"); return None

    def wait_for_idle(self):
        self.log("Waiting for move to complete...")
        while not self.stop_event.is_set():
            status = self.send_command("/")
            if status == 'N': self.log("Move complete."); return
            if status is None: raise ConnectionError("Lost connection.")
            time.sleep(0.05)

    def move_absolute(self, x_mm: float, y_mm: float):
        self.send_command(f"M X={int(x_mm * UNITS_MM_TO_DEVICE)} Y={int(y_mm * UNITS_MM_TO_DEVICE)}")
    def set_speed(self, speed_mms: float): self.send_command(f"S X={speed_mms} Y={speed_mms}")
    def halt(self): self.send_command(chr(92))

    def run_scan_step_and_measure(self, params: dict, line_callback: Callable):
        self.is_running_scan = True; self.stop_event.clear()
        self.log("--- Starting Scan (Step-and-Measure Mode) ---")
        try:
            steps_x, steps_y = int(params['steps_x']), int(params['steps_y'])
            x_coords = np.linspace(params['start_x'], params['end_x'], steps_x)
            y_coords = np.linspace(params['start_y'], params['end_y'], steps_y)
            results = np.full((steps_y, steps_x), np.nan)

            self.set_speed(params['speed'])
            
            for i, y in enumerate(y_coords):
                current_x_coords = x_coords if i % 2 == 0 else x_coords[::-1]
                for j, x in enumerate(current_x_coords):
                    if self.stop_event.is_set(): raise InterruptedError
                    self.move_absolute(x, y); self.wait_for_idle()
                    time.sleep(params['dwell']) # ЗАГЛУШКА: Сбор данных
                    value = np.random.rand() * 100 # Имитация сигнала
                    col_idx = j if i % 2 == 0 else (steps_x - 1 - j)
                    results[i, col_idx] = value
                
                if line_callback: line_callback(results.copy())
            self.log("--- Scan Completed Successfully ---")
        except InterruptedError: self.log("--- Scan Stopped by User ---"); self.halt()
        except Exception as e: self.log(f"--- ERROR during scan: {e} ---"); self.halt()
        finally: self.is_running_scan = False

    def stop_scan(self): self.log("Stop signal sent."); self.stop_event.set()


class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Interactive Scan Platform v3")
        self.root.geometry("1000x800") 
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.controller = MS2000Controller(self.log_message)
        self.scan_thread: Optional[threading.Thread] = None
        self.scan_area_patch: Optional[patches.Rectangle] = None
        self._create_widgets()
        self.update_calculated_fields()

    def _create_widgets(self):
        # --- НОВАЯ КОМПОНОВКА: ВЕРХ И НИЗ ---
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        bottom_frame = ttk.Frame(self.root, padding=10)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, expand=False)

        # --- ВЕРХНЯЯ ЧАСТЬ: КОНТРОЛЛЕРЫ И КАРТА ---
        controls_frame = ttk.Frame(top_frame)
        controls_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10), anchor='n')

        plot_frame = ttk.Frame(top_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # --- НИЖНЯЯ ЧАСТЬ: ЛОГ ---
        log_frame = ttk.LabelFrame(bottom_frame, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_area = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=10)
        self.log_area.pack(fill=tk.BOTH, expand=True)

        # --- ЗАПОЛНЕНИЕ ПАНЕЛИ УПРАВЛЕНИЯ (controls_frame) ---
        conn_frame = ttk.LabelFrame(controls_frame, text="1. Connection", padding=10); conn_frame.pack(fill=tk.X, pady=5)
        scan_param_frame = ttk.LabelFrame(controls_frame, text="2. Scan Parameters", padding=10); scan_param_frame.pack(fill=tk.X, pady=5)
        scan_control_frame = ttk.LabelFrame(controls_frame, text="3. Scan Control", padding=10); scan_control_frame.pack(fill=tk.X, pady=5)
        
        self.conn_entries = {}
        for i, (text, val) in enumerate({"Port": "COM4", "Baudrate": "9600"}.items()):
            ttk.Label(conn_frame, text=f"{text}:").grid(row=i, column=0, sticky="w"); entry = ttk.Entry(conn_frame, width=12); entry.insert(0, val); entry.grid(row=i, column=1, sticky="ew"); self.conn_entries[text.lower()] = entry
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect); self.connect_button.grid(row=2, column=0, columnspan=2, pady=5, sticky="ew")
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED); self.disconnect_button.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew")
        self.scan_entries = {}
        scan_fields = {"start_x": ("Start X (mm)", "10.0"), "start_y": ("Start Y (mm)", "10.0"), "steps_x": ("Points X", "50"), "steps_y": ("Points Y", "50"), "step_x":  ("Step X (mm)", "0.1"), "step_y":  ("Step Y (mm)", "0.1"), "speed":   ("Speed (mm/s)", "2.0"), "dwell":   ("Dwell (s)", "0.01")}
        i = 0
        for key, (label, value) in scan_fields.items():
            ttk.Label(scan_param_frame, text=label + ":").grid(row=i, column=0, sticky="w", pady=2); entry = ttk.Entry(scan_param_frame, width=10); entry.insert(0, value); entry.grid(row=i, column=1, sticky="ew", padx=5); entry.bind("<KeyRelease>", self.update_calculated_fields); self.scan_entries[key] = entry; i += 1
        self.calc_labels = {}
        for label, key in {"Area": "area"}.items():
            ttk.Label(scan_param_frame, text=label + ":").grid(row=i, column=0, sticky="w", pady=2); lbl = ttk.Label(scan_param_frame, text="0.0 mm", relief="sunken", width=10); lbl.grid(row=i, column=1, sticky="ew", padx=5); self.calc_labels[key] = lbl; i += 1
        self.start_scan_button = ttk.Button(scan_control_frame, text="Start Scan", command=self.start_scan, state=tk.DISABLED); self.start_scan_button.pack(fill=tk.X, pady=5)
        self.stop_scan_button = ttk.Button(scan_control_frame, text="Stop Scan", command=self.stop_scan, state=tk.DISABLED); self.stop_scan_button.pack(fill=tk.X)
        
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self._setup_main_plot() # Первичная отрисовка "карты местности"

    def _setup_main_plot(self):
        """Рисует начальную пустую карту 'Field of View'."""
        self.ax.clear()
        self.ax.set_xlim(0, FIELD_OF_VIEW_SIZE)
        self.ax.set_ylim(0, FIELD_OF_VIEW_SIZE)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_title("Field of View")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.grid(True, linestyle='--', alpha=0.6)
        if hasattr(self, 'colorbar'): self.colorbar.remove()
        self.canvas.draw()

    def update_calculated_fields(self, event=None):
        """Обновляет вычисляемые поля и рисует прямоугольник ROI."""
        try:
            params = self.get_scan_params(validate_integrality=False) # Не требуем целостности для превью
            if not params: return

            area_x = (params['steps_x'] - 1) * params['step_x']
            area_y = (params['steps_y'] - 1) * params['step_y']

            self.calc_labels["area"].config(text=f"{abs(area_x):.2f} x {abs(area_y):.2f} mm")

            if self.scan_area_patch:
                self.scan_area_patch.remove() 
            
            self.scan_area_patch = patches.Rectangle(
                (params['start_x'], params['start_y']),
                area_x, area_y,
                linewidth=1.5, edgecolor='r', facecolor='red', alpha=0.2
            )
            self.ax.add_patch(self.scan_area_patch)
            self.canvas.draw()

        except (ValueError, TypeError):
             self.calc_labels["area"].config(text="Error")

    def start_scan(self):
        if self.controller.is_running_scan: self.log_message("Scan is already running."); return
        params = self.get_scan_params()
        if params is None: return

        self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.NORMAL)
        
        # Перед запуском, создаем пустую NaN матрицу и отрисовываем ее
        nan_data = np.full((int(params['steps_y']), int(params['steps_x'])), np.nan)
        self.plot_scan_data(nan_data)

        def line_update_callback(partial_data):
            self.root.after(0, self.plot_scan_data, partial_data)

        self.scan_thread = threading.Thread(
            target=self.controller.run_scan_step_and_measure,
            args=(params, line_update_callback), daemon=True
        )
        self.scan_thread.start()
        self.check_scan_thread()

    def plot_scan_data(self, data: np.ndarray):
        """Отрисовывает карту данных сканирования."""
        params = self.get_scan_params(validate_integrality=False)
        if not params: return
        
        # Мы не очищаем карту, а рисуем поверх `imshow`
        extent = [
            params['start_x'], params['start_x'] + (params['steps_x']-1)*params['step_x'],
            params['start_y'], params['start_y'] + (params['steps_y']-1)*params['step_y']
        ]
        
        # Если это первый вызов для скана, создаем объект imshow
        if not hasattr(self, 'scan_image') or self.scan_image is None:
            self.scan_image = self.ax.imshow(data, cmap='viridis', origin='lower', extent=extent, interpolation='none')
            self.colorbar = self.fig.colorbar(self.scan_image, ax=self.ax)
        else: # Иначе, просто обновляем данные
            self.scan_image.set_data(data)
            self.scan_image.set_extent(extent)
            # Обновляем пределы colorbar
            if np.all(np.isnan(data)): # если все еще NaN
                self.scan_image.set_clim(0, 1)
            else:
                self.scan_image.set_clim(np.nanmin(data), np.nanmax(data))
                self.colorbar.update_normal(self.scan_image)

        self.canvas.draw()
        
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive():
            self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            # После завершения сканирования возвращаем пустую карту
            if self.controller.is_connected():
                self.start_scan_button.config(state=tk.NORMAL)
            # Убираем старое изображение скана, чтобы подготовиться к новому
            if hasattr(self, 'scan_image') and self.scan_image:
                self.scan_image.remove()
                self.scan_image = None
            self._setup_main_plot()
            self.update_calculated_fields() # Перерисовываем ROI
    
    def get_scan_params(self, validate_integrality=True) -> Optional[dict]:
        params = {}
        try:
            for key, entry in self.scan_entries.items(): params[key] = float(entry.get())
            if validate_integrality:
                if params['steps_x'] % 1 != 0 or params['steps_y'] % 1 != 0: self.log_message("ERROR: Points must be integers."); return None
                if params['steps_x'] < 1 or params['steps_y'] < 1: self.log_message("ERROR: Points must be >= 1."); return None
                params['end_x'] = params['start_x'] + (params['steps_x'] - 1) * params['step_x']
                params['end_y'] = params['start_y'] + (params['steps_y'] - 1) * params['step_y']
            return params
        except (ValueError, tk.TclError): self.log_message("ERROR: Invalid parameter."); return None
    def connect(self):
        if self.controller.connect(self.conn_entries['port'].get(), int(self.conn_entries['baudrate'].get())):
            self.connect_button.config(state=tk.DISABLED); self.disconnect_button.config(state=tk.NORMAL); self.start_scan_button.config(state=tk.NORMAL)
    def disconnect(self):
        self.controller.disconnect(); self.connect_button.config(state=tk.NORMAL); self.disconnect_button.config(state=tk.DISABLED); self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.DISABLED)
    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.controller.stop_scan()
    def on_closing(self): self.log_message("Closing..."); self.controller.disconnect(); self.root.destroy()
    def log_message(self, message: str):
        def append(): self.log_area.configure(state='normal'); self.log_area.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n"); self.log_area.configure(state='disabled'); self.log_area.see(tk.END)
        self.root.after(0, append)

if __name__ == "__main__":
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()