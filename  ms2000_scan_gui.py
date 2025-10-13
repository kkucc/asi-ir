import tkinter as tk
from tkinter import ttk
import serial
import time
import threading
import numpy as np
from typing import Optional, Callable

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.cm as cm
from matplotlib.patches import Rectangle

UNITS_MM_TO_DEVICE = 10000
FIELD_OF_VIEW_SIZE = 100

class MS2000Controller:
    # Этот класс не менялся, он работает корректно.
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None; self.is_running_scan = False
        self.stop_event = threading.Event(); self.log = log_callback
        self.lock = threading.Lock()
    def connect(self, port: str, baudrate: int) -> bool:
        if self.is_connected(): return True
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(0.2); self._set_high_precision()
            self.log(f"INFO: Successfully connected to MS-2000 on {port}.")
            return True
        except serial.SerialException as e:
            self.log(f"ERROR: Failed to connect: {e}"); self.ser = None; return False
    def disconnect(self):
        if self.is_running_scan: self.stop_scan()
        if self.ser and self.ser.is_open: self.ser.close(); self.log("INFO: Disconnected.")
        self.ser = None
    def is_connected(self) -> bool: return self.ser is not None and self.ser.is_open
    def _set_high_precision(self):
        if not self.ser: return
        try: self.ser.reset_input_buffer(); self.ser.write(bytes([255, 72])); time.sleep(0.1)
        except Exception: pass
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
        while not self.stop_event.is_set():
            status = self.send_command("/")
            if status == 'N': return
            if status is None: raise ConnectionError("Lost connection.")
            time.sleep(0.05)
    def move_absolute(self, x, y): self.send_command(f"M X={int(x*UNITS_MM_TO_DEVICE)} Y={int(y*UNITS_MM_TO_DEVICE)}")
    def set_speed(self, s): self.send_command(f"S X={s} Y={s}")
    def halt(self): self.send_command(chr(92))
    def run_scan(self, params, line_callback):
        self.is_running_scan = True; self.stop_event.clear()
        self.log("INFO: --- Starting Scan ---")
        try:
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
                    time.sleep(params['dwell'])
                    value = np.random.rand() * 100 if np.sqrt((x-50)**2 + (y-50)**2) < 10 else np.random.rand()*10
                    col_idx = j if i % 2 == 0 else (steps_x - 1 - j)
                    results[i, col_idx] = value
                if line_callback: line_callback(results.copy())
            self.log("INFO: --- Scan Completed ---")
        except InterruptedError: self.log("INFO: --- Scan Stopped ---"); self.halt()
        except Exception as e: self.log(f"ERROR: --- Scan Failed: {e} ---"); self.halt()
        finally: self.is_running_scan = False
    def stop_scan(self): self.log("INFO: Stop signal sent."); self.stop_event.set()

class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Minimap Cockpit v6")
        self.root.geometry("680x500")
        self.controller = MS2000Controller(lambda msg: print(f"{time.strftime('%H:%M:%S')} - {msg}"))
        self.scan_thread: Optional[threading.Thread] = None
        self.minimap_extents = [0, FIELD_OF_VIEW_SIZE, 0, FIELD_OF_VIEW_SIZE]
        self._create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.columnconfigure(0, weight=1); main_frame.columnconfigure(1, weight=0)
        main_frame.rowconfigure(1, weight=1)

        top_left_frame = ttk.Frame(main_frame)
        top_right_frame = ttk.Frame(main_frame, width=250, height=250)
        bottom_frame = ttk.Frame(main_frame)
        
        top_left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        top_right_frame.grid(row=0, column=1, rowspan=2, sticky="ne")
        bottom_frame.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        top_right_frame.grid_propagate(False)

        # 1. Top Left: Connection and Scan Control
        conn_frame = ttk.LabelFrame(top_left_frame, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X)
        scan_ctrl_frame = ttk.LabelFrame(top_left_frame, text="Scan Control", padding=10)
        scan_ctrl_frame.pack(fill=tk.X, pady=(10,0))
        
        self.conn_entries={}
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w", pady=2)
        port_entry = ttk.Entry(conn_frame, width=8); port_entry.insert(0, "COM4")
        port_entry.grid(row=0, column=1); self.conn_entries['port'] = port_entry
        ttk.Label(conn_frame, text="Baud:").grid(row=1, column=0, sticky="w", pady=2)
        baud_entry = ttk.Entry(conn_frame, width=8); baud_entry.insert(0, "9600")
        baud_entry.grid(row=1, column=1); self.conn_entries['baudrate'] = baud_entry
        btn_frame = ttk.Frame(conn_frame); btn_frame.grid(row=0, column=2, rowspan=2, padx=10)
        self.connect_button = ttk.Button(btn_frame, text="Connect", command=self.connect); self.connect_button.pack(fill=tk.X, pady=2)
        self.disconnect_button = ttk.Button(btn_frame, text="Disconnect", command=self.disconnect, state=tk.DISABLED); self.disconnect_button.pack(fill=tk.X, pady=2)

        self.start_scan_button = ttk.Button(scan_ctrl_frame, text="Start Scan", command=self.start_scan, state=tk.DISABLED); self.start_scan_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,5))
        self.stop_scan_button = ttk.Button(scan_ctrl_frame, text="Stop Scan", command=self.stop_scan, state=tk.DISABLED); self.stop_scan_button.pack(side=tk.LEFT, expand=True, fill=tk.X)

        # 2. Top Right: Minimap and controls
        self.fig = Figure(figsize=(2.5, 2.5), dpi=100); self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=top_right_frame); self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.setup_minimap()
        map_controls_frame = ttk.Frame(top_right_frame); map_controls_frame.pack(fill=tk.X, pady=5)
        ttk.Label(map_controls_frame, text="Map View:").pack(side=tk.LEFT)
        ttk.Button(map_controls_frame, text="+", width=3, command=self.zoom_in).pack(side=tk.LEFT, padx=2)
        ttk.Button(map_controls_frame, text="-", width=3, command=self.zoom_out).pack(side=tk.LEFT, padx=2)
        ttk.Button(map_controls_frame, text="Reset", command=self.reset_zoom).pack(side=tk.LEFT, padx=2)
        
        # 3. Scan Parameters with NEW LAYOUT
        param_frame = ttk.LabelFrame(bottom_frame, text="Scan Parameters", padding=10)
        param_frame.pack(expand=True, fill=tk.BOTH)
        self.scan_entries={}
        # --- ИСПРАВЛЕННАЯ ЛОГИКА РАЗМЕЩЕНИЯ ---
        # Labels for columns
        ttk.Label(param_frame, text="X-Axis", font='-weight bold').grid(row=0, column=0, columnspan=2, pady=(0, 5))
        ttk.Label(param_frame, text="Y-Axis", font='-weight bold').grid(row=0, column=3, columnspan=2, pady=(0, 5))
        
        scan_fields_x = {"start_x":("Start (mm)","45.0"), "steps_x":("Points","50"), "step_x":("Step (mm)","0.2")}
        scan_fields_y = {"start_y":("Start (mm)","45.0"), "steps_y":("Points","50"), "step_y":("Step (mm)","0.2")}
        
        for i, (k, (l, v)) in enumerate(scan_fields_x.items()):
            ttk.Label(param_frame, text=l+":").grid(row=i+1, column=0, sticky="w", pady=2)
            e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=i+1, column=1, padx=5); e.bind("<KeyRelease>", self.update_scan_area_preview); self.scan_entries[k]=e
        
        for i, (k, (l, v)) in enumerate(scan_fields_y.items()):
            ttk.Label(param_frame, text=l+":").grid(row=i+1, column=3, sticky="w", pady=2)
            e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=i+1, column=4, padx=5); e.bind("<KeyRelease>", self.update_scan_area_preview); self.scan_entries[k]=e
        
        param_frame.columnconfigure(2, weight=1, minsize=20) # Spacer column

        ttk.Separator(param_frame, orient='horizontal').grid(row=4, column=0, columnspan=5, sticky='ew', pady=10)
        
        # General parameters at the bottom
        scan_fields_general = {"speed":("Speed (mm/s)","2.0"), "dwell":("Dwell (s)","0.01")}
        for i, (k, (l, v)) in enumerate(scan_fields_general.items()):
            ttk.Label(param_frame, text=l+":").grid(row=5, column=i*3, sticky="w", pady=2)
            e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=5, column=i*3+1, padx=5); self.scan_entries[k]=e

    def setup_minimap(self):
        self.ax.clear(); self.ax.set_xticks([]); self.ax.set_yticks([]); self.ax.set_facecolor('#cccccc'); self.ax.set_aspect('equal', adjustable='box')
        if hasattr(self, 'scan_image'): del self.scan_image
        if hasattr(self, 'scan_area_patch'): del self.scan_area_patch
        self.update_minimap_view()
    def update_minimap_view(self): self.ax.set_xlim(self.minimap_extents[0], self.minimap_extents[1]); self.ax.set_ylim(self.minimap_extents[2], self.minimap_extents[3]); self.canvas.draw()
    def zoom_in(self): x0,x1,y0,y1=self.minimap_extents;cx,cy=(x0+x1)/2,(y0+y1)/2;w,h=(x1-x0)/2,(y1-y0)/2;self.minimap_extents=[cx-w/2,cx+w/2,cy-h/2,cy+h/2];self.update_minimap_view()
    def zoom_out(self): x0,x1,y0,y1=self.minimap_extents;cx,cy=(x0+x1)/2,(y0+y1)/2;w,h=(x1-x0)*2,(y1-y0)*2;self.minimap_extents=[max(0,cx-w/2),min(FIELD_OF_VIEW_SIZE,cx+w/2),max(0,cy-h/2),min(FIELD_OF_VIEW_SIZE,cy+h/2)];self.update_minimap_view()
    def reset_zoom(self): self.minimap_extents = [0, FIELD_OF_VIEW_SIZE, 0, FIELD_OF_VIEW_SIZE]; self.update_minimap_view()
    def update_scan_area_preview(self, event=None):
        if hasattr(self, 'scan_area_patch') and self.scan_area_patch: self.scan_area_patch.remove()
        try:
            params = self.get_scan_params(validate_integrality=False); width = (params['steps_x'] - 1) * params['step_x']; height = (params['steps_y'] - 1) * params['step_y']
            self.scan_area_patch = self.ax.add_patch(Rectangle((params['start_x'], params['start_y']), width, height, lw=1, ec='black', fc='black', alpha=0.5)); self.canvas.draw()
        except (ValueError, TypeError, KeyError): pass
    def start_scan(self):
        if self.controller.is_running_scan: return
        params = self.get_scan_params();
        if params is None: return
        self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.NORMAL)
        nan_data = np.full((int(params['steps_y']), int(params['steps_x'])), np.nan); self.plot_scan_data(nan_data)
        self.scan_thread = threading.Thread(target=self.controller.run_scan, args=(params, self.line_update_callback), daemon=True); self.scan_thread.start()
        self.check_scan_thread()
    def line_update_callback(self, partial_data): self.root.after(0, self.plot_scan_data, partial_data)
    def plot_scan_data(self, data: np.ndarray):
        params = self.get_scan_params(validate_integrality=False);
        if not params: return
        extent = [params['start_x'], params['end_x'], params['start_y'], params['end_y']]
        cmap = cm.get_cmap('viridis').copy(); cmap.set_bad(color='black')
        if not hasattr(self, 'scan_image') or self.scan_image is None:
            self.scan_image = self.ax.imshow(data, cmap=cmap, origin='lower', extent=extent, interpolation='none', vmin=0, vmax=100)
        else:
            self.scan_image.set_data(data); self.scan_image.set_extent(extent)
            if not np.all(np.isnan(data)): self.scan_image.set_clim(np.nanmin(data), np.nanmax(data))
        self.canvas.draw()
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected(): self.start_scan_button.config(state=tk.NORMAL)
            self.setup_minimap(); self.update_scan_area_preview()
    def get_scan_params(self, validate_integrality=True):
        try:
            params = {k: float(e.get()) for k,e in self.scan_entries.items()}
            if validate_integrality and (params['steps_x'] % 1 != 0 or params['steps_y'] % 1 != 0 or params['steps_x'] < 1 or params['steps_y'] < 1): print("ERROR: Points must be integers >= 1."); return None
            params['end_x'] = params['start_x'] + (params['steps_x'] - 1) * params['step_x']
            params['end_y'] = params['start_y'] + (params['steps_y'] - 1) * params['step_y']
            return params
        except (ValueError, tk.TclError): print("ERROR: Invalid scan parameter."); return None
    def connect(self):
        if self.controller.connect(self.conn_entries['port'].get(), int(self.conn_entries['baudrate'].get())):
            self.connect_button.config(state=tk.DISABLED); self.disconnect_button.config(state=tk.NORMAL); self.start_scan_button.config(state=tk.NORMAL)
    def disconnect(self):
        self.controller.disconnect(); self.connect_button.config(state=tk.NORMAL); self.disconnect_button.config(state=tk.DISABLED); self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.DISABLED)
    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.controller.stop_scan()
    def on_closing(self): self.controller.disconnect(); self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()