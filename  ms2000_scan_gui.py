import tkinter as tk
from tkinter import ttk
import serial
import time
import threading
import numpy as np
from typing import Optional, Callable, Any
import abc

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.cm as cm
from matplotlib.patches import Rectangle


class AcquisitionDevice(abc.ABC):
    """Абстрактный 'контракт' для любого измерительного устройства."""
    @abc.abstractmethod
    def connect(self) -> bool: pass
    @abc.abstractmethod
    def disconnect(self) -> bool: pass
    @abc.abstractmethod
    def acquire(self, dwell_time: float) -> Any: pass
    def __str__(self): return self.__class__.__name__

class SmartDummySignal(AcquisitionDevice):
    """'Умная' заглушка, имитирующая яркий объект на темном фоне."""
    def __init__(self, center_x=50, center_y=50, radius=15):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        print(f"INFO: {self} initialized with a signal at ({center_x}, {center_y})")

    def connect(self) -> bool: return True
    def disconnect(self) -> bool: return True

    def acquire(self, dwell_time: float) -> float:
        # Эта заглушка не использует dwell_time, но он нужен для совместимости
        # В реальном приборе здесь будет ожидание
        
        # Получаем текущие координаты от контроллера
        current_x, current_y = app.controller.get_last_known_position()
        
        distance = np.sqrt((current_x - self.center_x)**2 + (current_y - self.center_y)**2)
        if distance < self.radius:
            return 90 + np.random.rand() * 10 # Высокий сигнал
        else:
            return 10 + np.random.rand() * 10 # Низкий фон

class RandomNoiseDevice(AcquisitionDevice):
    """Простая заглушка, генерирующая случайный шум."""
    def connect(self) -> bool: return True
    def disconnect(self) -> bool: return True
    def acquire(self, dwell_time: float) -> float:
        time.sleep(dwell_time) 
        return np.random.rand() * 100

# УПРАВЛЕНИЯ СТОЛИКОМ 

UNITS_MM_TO_DEVICE = 10000

class MS2000Controller:
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None; self.is_running_scan = False
        self.stop_event = threading.Event(); self.log = log_callback
        self.lock = threading.Lock()
        self.last_x = 0.0; self.last_y = 0.0

    def get_last_known_position(self): return self.last_x, self.last_y
    def connect(self, port, baud):
        if self.is_connected(): return True
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0); time.sleep(0.2); self._set_high_precision()
            self.log(f"INFO: Connected to MS-2000 on {port}."); return True
        except serial.SerialException as e: self.log(f"ERROR: {e}"); self.ser=None; return False
    def disconnect(self):
        if self.is_running_scan: self.stop_scan()
        if self.ser and self.ser.is_open: self.ser.close(); self.log("INFO: Disconnected.")
        self.ser = None
    def is_connected(self): return self.ser is not None and self.ser.is_open
    def _set_high_precision(self):
        if not self.ser: return
        try: self.ser.reset_input_buffer(); self.ser.write(bytes([255, 72])); time.sleep(0.1)
        except Exception: pass
    def send_command(self, cmd):
        if not self.is_connected(): return None
        with self.lock:
            try:
                self.log(f"CMD > {cmd}"); self.ser.reset_input_buffer(); self.ser.write(f"{cmd}\r".encode('ascii'))
                return self.ser.read_until(b'\r\n').decode('ascii').strip()
            except Exception as e: self.log(f"ERROR: {e}"); return None
    def wait_for_idle(self):
        while not self.stop_event.is_set():
            if self.send_command("/") == 'N': return
            time.sleep(0.05)
    def move_absolute(self, x, y): self.last_x, self.last_y = x, y; self.send_command(f"M X={int(x*UNITS_MM_TO_DEVICE)} Y={int(y*UNITS_MM_TO_DEVICE)}")
    def set_speed(self, s): self.send_command(f"S X={s} Y={s}")
    def halt(self): self.send_command(chr(92))

    def run_scan(self, params, device: AcquisitionDevice, line_callback):
        self.is_running_scan = True; self.stop_event.clear()
        self.log(f"INFO: --- Starting Scan with {device} ---")
        try:
            device.connect()
            steps_x, steps_y = int(params['steps_x']), int(params['steps_y'])
            x_coords = np.linspace(params['start_x'], params['end_x'], steps_x)
            y_coords = np.linspace(params['start_y'], params['end_y'], steps_y)
            results = np.full((steps_y, steps_x), np.nan)
            self.set_speed(params['speed'])
            for i, y in enumerate(y_coords):
                xs = x_coords if i % 2 == 0 else x_coords[::-1]
                for j, x in enumerate(xs):
                    if self.stop_event.is_set(): raise InterruptedError
                    self.move_absolute(x, y); self.wait_for_idle()
                    value = device.acquire(params['dwell'])
                    col_idx = j if i % 2 == 0 else (steps_x - 1 - j)
                    results[i, col_idx] = value
                if line_callback: line_callback(results.copy(), i, steps_y)
            self.log("INFO: --- Scan Completed ---")
        except InterruptedError: self.log("INFO: --- Scan Stopped ---"); self.halt()
        except Exception as e: self.log(f"ERROR: --- Scan Failed: {e} ---"); self.halt()
        finally: device.disconnect(); self.is_running_scan = False
    def stop_scan(self): self.log("INFO: Stop signal sent."); self.stop_event.set()


class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Cockpit v7")
        self.root.geometry("720x520")
        self.controller = MS2000Controller(lambda msg: print(f"{time.strftime('%H:%M:%S')} - {msg}"))
        self.scan_thread: Optional[threading.Thread] = None
        
        self.available_devices = [SmartDummySignal(), RandomNoiseDevice()]
        
        self._create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding=10); main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.columnconfigure(0, weight=1); main_frame.columnconfigure(1, weight=0); main_frame.rowconfigure(1, weight=1)
        
        top_left_frame = ttk.Frame(main_frame); top_right_frame = ttk.Frame(main_frame, width=280, height=300); bottom_frame = ttk.Frame(main_frame)
        top_left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10)); top_right_frame.grid(row=0, column=1, rowspan=2, sticky="ne"); bottom_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        top_right_frame.grid_propagate(False)

        # 1. Top Left
        conn_frame = ttk.LabelFrame(top_left_frame, text="Connection", padding=10); conn_frame.pack(fill=tk.X)
        scan_ctrl_frame = ttk.LabelFrame(top_left_frame, text="Scan Control", padding=10); scan_ctrl_frame.pack(fill=tk.X, pady=(10,0))
        self.conn_entries={}; ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w", pady=2); port_entry = ttk.Entry(conn_frame, width=8); port_entry.insert(0, "COM4"); port_entry.grid(row=0, column=1); self.conn_entries['port'] = port_entry
        ttk.Label(conn_frame, text="Baud:").grid(row=1, column=0, sticky="w", pady=2); baud_entry = ttk.Entry(conn_frame, width=8); baud_entry.insert(0, "9600"); baud_entry.grid(row=1, column=1); self.conn_entries['baudrate'] = baud_entry
        btn_frame = ttk.Frame(conn_frame); btn_frame.grid(row=0, column=2, rowspan=2, padx=10); self.connect_button = ttk.Button(btn_frame, text="Connect", command=self.connect); self.connect_button.pack(fill=tk.X, pady=2); self.disconnect_button = ttk.Button(btn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED); self.disconnect_button.pack(fill=tk.X, pady=2)
        ttk.Label(scan_ctrl_frame, text="Device:").pack(fill=tk.X)
        self.device_combobox = ttk.Combobox(scan_ctrl_frame, values=[str(d) for d in self.available_devices], state="readonly"); self.device_combobox.current(0); self.device_combobox.pack(fill=tk.X, pady=(0,5))
        self.start_scan_button = ttk.Button(scan_ctrl_frame, text="Start Scan", command=self.start_scan, state=tk.DISABLED); self.start_scan_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,5))
        self.stop_scan_button = ttk.Button(scan_ctrl_frame, text="Stop Scan", command=self.stop_scan, state=tk.DISABLED); self.stop_scan_button.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # 2. Top Right: Minimap and controls
        map_settings_frame = ttk.LabelFrame(top_right_frame, text="Map Settings", padding=10); map_settings_frame.pack(fill=tk.X)
        self.map_entries = {}
        ttk.Label(map_settings_frame, text="Width (mm):").grid(row=0, column=0); e = ttk.Entry(map_settings_frame, width=6); e.insert(0, "100"); e.grid(row=0, column=1, padx=5); self.map_entries['w'] = e
        ttk.Label(map_settings_frame, text="Height (mm):").grid(row=0, column=2); e = ttk.Entry(map_settings_frame, width=6); e.insert(0, "100"); e.grid(row=0, column=3, padx=5); self.map_entries['h'] = e
        ttk.Button(map_settings_frame, text="Apply", command=self.apply_map_settings).grid(row=0, column=4, padx=5)

        self.fig = Figure(figsize=(2.5, 2.5), dpi=100); self.ax = self.fig.add_subplot(111); self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=top_right_frame); self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=5)
        self.setup_minimap()

        map_controls_frame = ttk.Frame(top_right_frame); map_controls_frame.pack(fill=tk.X, pady=5)
        ttk.Label(map_controls_frame, text="Map View:").pack(side=tk.LEFT); ttk.Button(map_controls_frame, text="+", width=3, command=self.zoom_in).pack(side=tk.LEFT, padx=2); ttk.Button(map_controls_frame, text="-", width=3, command=self.zoom_out).pack(side=tk.LEFT, padx=2); ttk.Button(map_controls_frame, text="Reset", command=self.reset_zoom).pack(side=tk.LEFT, padx=2)
        
        # 3. Bottom: Scan Parameters
        param_frame = ttk.LabelFrame(bottom_frame, text="Scan Parameters", padding=10); param_frame.pack(expand=True, fill=tk.BOTH)
        self.scan_entries={}; ttk.Label(param_frame, text="X-Axis", font='-weight bold').grid(row=0, column=0, columnspan=2, pady=(0, 5)); ttk.Label(param_frame, text="Y-Axis", font='-weight bold').grid(row=0, column=3, columnspan=2, pady=(0, 5))
        for i, (k,(l,v)) in enumerate({"start_x":("Start","45.0"), "steps_x":("Points","50"), "step_x":("Step","0.2")}.items()): ttk.Label(param_frame, text=l+":").grid(row=i+1, column=0, sticky="w", pady=2); e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=i+1, column=1, padx=5); e.bind("<KeyRelease>", self.update_scan_area_preview); self.scan_entries[k]=e
        for i, (k,(l,v)) in enumerate({"start_y":("Start","45.0"), "steps_y":("Points","50"), "step_y":("Step","0.2")}.items()): ttk.Label(param_frame, text=l+":").grid(row=i+1, column=3, sticky="w", pady=2); e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=i+1, column=4, padx=5); e.bind("<KeyRelease>", self.update_scan_area_preview); self.scan_entries[k]=e
        param_frame.columnconfigure(2, weight=1, minsize=20); ttk.Separator(param_frame, orient='horizontal').grid(row=4, column=0, columnspan=5, sticky='ew', pady=10)
        for i, (k,(l,v)) in enumerate({"speed":("Speed","2.0"), "dwell":("Dwell","0.01")}.items()): ttk.Label(param_frame, text=l+":").grid(row=5, column=i*3, sticky="w", pady=2); e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=5, column=i*3+1, padx=5); self.scan_entries[k]=e

    def setup_minimap(self):
        self.ax.clear(); self.ax.set_xticks([]); self.ax.set_yticks([]); self.ax.set_facecolor('#cccccc'); self.ax.set_aspect('equal', adjustable='box')
        if hasattr(self, 'scan_image'): del self.scan_image
        if hasattr(self, 'scan_area_patch'): self.scan_area_patch=None
        if hasattr(self, 'progress_line') and self.progress_line: self.progress_line.remove(); self.progress_line = None
        self.apply_map_settings()
    def apply_map_settings(self):
        try: w=float(self.map_entries['w'].get()); h=float(self.map_entries['h'].get()); self.minimap_extents=[0,w,0,h]; self.reset_zoom()
        except: self.controller.log("ERROR: Invalid map dimensions.")
    def update_minimap_view(self): self.ax.set_xlim(self.minimap_extents[0], self.minimap_extents[1]); self.ax.set_ylim(self.minimap_extents[2], self.minimap_extents[3]); self.canvas.draw()
    def zoom_in(self): x0,x1,y0,y1=self.minimap_extents;cx,cy=(x0+x1)/2,(y0+y1)/2;w,h=(x1-x0)/2,(y1-y0)/2;self.minimap_extents=[cx-w/2,cx+w/2,cy-h/2,cy+h/2];self.update_minimap_view()
    def zoom_out(self): x0,x1,y0,y1=self.minimap_extents;w,h=(x1-x0)*2,(y1-y0)*2;mw,mh=float(self.map_entries['w'].get()),float(self.map_entries['h'].get());self.minimap_extents=[max(0,cx-w/2),min(mw,cx+w/2),max(0,cy-h/2),min(mh,cy+h/2)];self.update_minimap_view()
    def reset_zoom(self): self.minimap_extents=[0,float(self.map_entries['w'].get()),0,float(self.map_entries['h'].get())]; self.update_minimap_view()
    def update_scan_area_preview(self, e=None):
        if hasattr(self,'scan_area_patch') and self.scan_area_patch: self.scan_area_patch.remove()
        try:
            params=self.get_scan_params(False); w=(params['steps_x']-1)*params['step_x']; h=(params['steps_y']-1)*params['step_y']
            self.scan_area_patch=self.ax.add_patch(Rectangle((params['start_x'],params['start_y']),w,h,lw=1,ec='black',fc='black',alpha=0.5));self.canvas.draw()
        except(ValueError,TypeError,KeyError):pass
    def start_scan(self):
        if self.controller.is_running_scan: return
        params = self.get_scan_params()
        device = next((d for d in self.available_devices if str(d)==self.device_combobox.get()), None)
        if params is None or device is None: self.controller.log("ERROR: Invalid params or no device selected."); return
        self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.NORMAL)
        nan_data = np.full((int(params['steps_y']), int(params['steps_x'])), np.nan); self.plot_scan_data(nan_data, 0, 0)
        self.scan_thread = threading.Thread(target=self.controller.run_scan, args=(params, device, self.line_update_callback), daemon=True); self.scan_thread.start()
        self.check_scan_thread()
    def line_update_callback(self, data, row, total_rows): self.root.after(0, self.plot_scan_data, data, row, total_rows)
    def plot_scan_data(self, data, row_index, total_rows):
        params = self.get_scan_params(False);
        if not params: return
        extent = [params['start_x'], params['end_x'], params['start_y'], params['end_y']]
        cmap=cm.get_cmap('viridis').copy();cmap.set_bad(color='black')
        if not hasattr(self, 'scan_image') or self.scan_image is None: self.scan_image = self.ax.imshow(data, cmap=cmap, origin='lower', extent=extent, interpolation='none', vmin=0, vmax=100)
        else: self.scan_image.set_data(data); self.scan_image.set_extent(extent);
        if not np.all(np.isnan(data)): self.scan_image.set_clim(np.nanmin(data), np.nanmax(data))
        if self.progress_line: self.progress_line.remove()
        step_y = (params['end_y'] - params['start_y'])/(total_rows-1) if total_rows>1 else 0
        self.progress_line = self.ax.axhline(y=params['start_y'] + row_index*step_y, color='yellow', lw=2, alpha=0.7)
        self.canvas.draw()
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected(): self.start_scan_button.config(state=tk.NORMAL)
            self.setup_minimap(); self.update_scan_area_preview()
    def get_scan_params(self, validate=True):
        try:
            p = {k: float(e.get()) for k,e in self.scan_entries.items()}
            if validate and (p['steps_x']%1!=0 or p['steps_y']%1!=0 or p['steps_x']<1 or p['steps_y']<1): print("ERROR: Points must be int >= 1"); return None
            p['end_x']=p['start_x']+(p['steps_x']-1)*p['step_x']; p['end_y']=p['start_y']+(p['steps_y']-1)*p['step_y']
            return p
        except(ValueError,tk.TclError): print("ERROR: Invalid param"); return None
    def connect(self):
        if self.controller.connect(self.conn_entries['port'].get(), int(self.conn_entries['baudrate'].get())):
            self.connect_button.config(state=tk.DISABLED);self.disconnect_button.config(state=tk.NORMAL);self.start_scan_button.config(state=tk.NORMAL)
    def disconnect(self): self.controller.disconnect();self.connect_button.config(state=tk.NORMAL);self.disconnect_button.config(state=tk.DISABLED);self.start_scan_button.config(state=tk.DISABLED);self.stop_scan_button.config(state=tk.DISABLED)
    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.controller.stop_scan()
    def on_closing(self): self.controller.disconnect(); self.root.destroy()

if __name__ == "__main__":
    app_instance = None
    def main():
        global app_instance
        root = tk.Tk()
        app_instance = StageControlApp(root)
        root.mainloop()
    main()