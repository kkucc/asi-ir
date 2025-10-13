import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import time
import threading
import numpy as np
from typing import Optional, Callable

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.cm as cm

# 1 mm = 1000 microns = 10000 tenths of a micron.
UNITS_MM_TO_DEVICE = 10000
FIELD_OF_VIEW_SIZE = 100 # Размер "миникарты" в мм

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
        self.log("Waiting for move...")
        while not self.stop_event.is_set():
            status = self.send_command("/")
            if status == 'N': self.log("Move complete."); return
            if status is None: raise ConnectionError("Lost connection.")
            time.sleep(0.05)
    def move_absolute(self, x_mm: float, y_mm: float): self.send_command(f"M X={int(x_mm * UNITS_MM_TO_DEVICE)} Y={int(y_mm * UNITS_MM_TO_DEVICE)}")
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
                    time.sleep(params['dwell'])
                    value = np.random.rand() * 100
                    col_idx = j if i % 2 == 0 else (steps_x - 1 - j)
                    results[i, col_idx] = value
                if line_callback: line_callback(results.copy())
            self.log("--- Scan Completed ---")
        except InterruptedError: self.log("--- Scan Stopped ---"); self.halt()
        except Exception as e: self.log(f"--- ERROR during scan: {e} ---"); self.halt()
        finally: self.is_running_scan = False
    def stop_scan(self): self.log("Stop signal sent."); self.stop_event.set()


class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Minimap Scan Platform v4")
        self.root.geometry("1100x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.controller = MS2000Controller(self.log_message)
        self.scan_thread: Optional[threading.Thread] = None
        self._create_widgets()

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 3-КОЛОНОЧНАЯ КОМПОНОВКА ---
        controls_frame = ttk.Frame(main_frame, width=250)
        controls_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10), anchor='n')
        controls_frame.pack_propagate(False) # Запрещаем сжиматься

        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        log_frame = ttk.Frame(main_frame)
        log_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        #КОЛОНКА 1: УПРАВЛЕНИЕ ---
        conn_frame = ttk.LabelFrame(controls_frame, text="1. Connection", padding=10); conn_frame.pack(fill=tk.X, pady=5)
        scan_param_frame = ttk.LabelFrame(controls_frame, text="2. Scan Parameters", padding=10); scan_param_frame.pack(fill=tk.X, pady=5)
        scan_control_frame = ttk.LabelFrame(controls_frame, text="3. Scan Control", padding=10); scan_control_frame.pack(fill=tk.X, pady=5)
        
        self.conn_entries = {}
        for i,(t,v) in enumerate({"Port":"COM4","Baudrate":"9600"}.items()): ttk.Label(conn_frame,text=f"{t}:").grid(row=i,column=0,sticky="w");e=ttk.Entry(conn_frame,width=12);e.insert(0,v);e.grid(row=i,column=1,sticky="ew");self.conn_entries[t.lower()]=e
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.connect); self.connect_button.grid(row=2,column=0,columnspan=2,pady=5,sticky="ew")
        self.disconnect_button = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED); self.disconnect_button.grid(row=3,column=0,columnspan=2,pady=5,sticky="ew")
        self.scan_entries = {}
        scan_fields = {"start_x":("Start X","10.0"), "start_y":("Start Y","10.0"), "steps_x":("Points X","50"), "steps_y":("Points Y","50"), "step_x":("Step X","0.1"), "step_y":("Step Y","0.1"), "speed":("Speed","1.0"), "dwell":("Dwell","0.01")}
        for i,(k,(l,v)) in enumerate(scan_fields.items()): ttk.Label(scan_param_frame,text=l+":").grid(row=i,column=0,sticky="w",pady=2);e=ttk.Entry(scan_param_frame,width=8);e.insert(0,v);e.grid(row=i,column=1,sticky="ew",padx=5);self.scan_entries[k]=e
        self.start_scan_button = ttk.Button(scan_control_frame,text="Start Scan",command=self.start_scan,state=tk.DISABLED);self.start_scan_button.pack(fill=tk.X, pady=5)
        self.stop_scan_button = ttk.Button(scan_control_frame,text="Stop Scan",command=self.stop_scan,state=tk.DISABLED);self.stop_scan_button.pack(fill=tk.X)

        # МИНИКАРТА ===
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.setup_minimap()

        # # === КОЛОНКА 3: ЛОГ ===
        # self.log_area = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=10)
        # self.log_area.pack(fill=tk.BOTH, expand=True)

    def setup_minimap(self):
        """Рисует пустую 'миникарту' без осей и лишних элементов."""
        self.ax.clear()
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)
        self.ax.spines['left'].set_visible(False)
        self.ax.set_facecolor('#e0e0e0') # Светло-серый фон
        
        # Задаем фиксированные пределы для нашей карты
        self.ax.set_xlim(0, FIELD_OF_VIEW_SIZE)
        self.ax.set_ylim(0, FIELD_OF_VIEW_SIZE)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_title("Field of View")
        
        if hasattr(self, 'scan_image'): del self.scan_image
        if hasattr(self, 'colorbar'): self.colorbar.remove(); del self.colorbar
        self.canvas.draw()

    def start_scan(self):
        if self.controller.is_running_scan: self.log_message("Scan is already running."); return
        params = self.get_scan_params()
        if params is None: return

        self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.NORMAL)

        # Перед сканированием рисуем черную область там, где будут данные
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
        params = self.get_scan_params(validate_integrality=False)
        if not params: return
        
        extent = [params['start_x'], params['end_x'], params['start_y'], params['end_y']]
        
        # Настраиваем палитру: viridis для данных, черный для NaN
        cmap = cm.get_cmap('viridis').copy()
        cmap.set_bad(color='black')
        
        if not hasattr(self, 'scan_image') or self.scan_image is None:
            self.scan_image = self.ax.imshow(data, cmap=cmap, origin='lower', extent=extent, interpolation='none', vmin=0, vmax=100)
            self.colorbar = self.fig.colorbar(self.scan_image, ax=self.ax, fraction=0.046, pad=0.04)
        else:
            self.scan_image.set_data(data)
            self.scan_image.set_extent(extent)
            if not np.all(np.isnan(data)):
                vmin, vmax = np.nanmin(data), np.nanmax(data)
                self.scan_image.set_clim(vmin if vmin != vmax else vmin - 1, vmax)
                self.colorbar.update_normal(self.scan_image)
        self.canvas.draw()
        
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive():
            self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected():
                self.start_scan_button.config(state=tk.NORMAL)
            # После завершения сканирования, очищаем карту для следующего раза
            self.setup_minimap()
    
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

    # (connect, disconnect, stop_scan, on_closing, log_message)
    def connect(self):
        if self.controller.connect(self.conn_entries['port'].get(), int(self.conn_entries['baudrate'].get())):
            self.connect_button.config(state=tk.DISABLED);self.disconnect_button.config(state=tk.NORMAL);self.start_scan_button.config(state=tk.NORMAL)
    def disconnect(self):
        self.controller.disconnect();self.connect_button.config(state=tk.NORMAL);self.disconnect_button.config(state=tk.DISABLED);self.start_scan_button.config(state=tk.DISABLED);self.stop_scan_button.config(state=tk.DISABLED)
    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.controller.stop_scan()
    def on_closing(self): self.log_message("Closing..."); self.controller.disconnect(); self.root.destroy()
    def log_message(self, message: str):
        def append(): self.log_area.configure(state='normal');self.log_area.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n");self.log_area.configure(state='disabled');self.log_area.see(tk.END)
        self.root.after(0, append)

if __name__ == "__main__":
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()