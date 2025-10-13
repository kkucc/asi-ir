import tkinter as tk
from tkinter import ttk, messagebox
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

UNITS_MM_TO_DEVICE = 10000
STAGE_X_MIN, STAGE_X_MAX = -34.0, 39.0
STAGE_Y_MIN, STAGE_Y_MAX = -34.0, 39.0

#  АРХИТЕКТУРА "ЗАГЛУШЕК" 

class AcquisitionDevice(abc.ABC):
    @abc.abstractmethod
    def acquire(self, dwell_time: float, x: float, y: float) -> Any: pass
    def __str__(self): return self.__class__.__name__

class SmartDummySignal(AcquisitionDevice):
    def acquire(self, dwell_time: float, x: float, y: float) -> float:
        center_x, center_y, radius = 10.0, 10.0, 5.0
        distance = np.sqrt((x - center_x)**2 + (y - center_y)**2)
        if distance < radius: return 90 + np.random.rand() * 10
        else: return 10 + np.random.rand() * 10

class RandomNoiseDevice(AcquisitionDevice):
    def acquire(self, dwell_time: float, x: float, y: float) -> float:
        time.sleep(dwell_time)
        return np.random.rand() * 100

# КЛАСС УПРАВЛЕНИЯ СТОЛИКОМ 

class MS2000Controller:
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None; self.is_running_scan = False
        self.stop_event = threading.Event(); self.log = log_callback; self.lock = threading.Lock()
    def is_connected(self) -> bool: return self.ser is not None and self.ser.is_open
    def connect(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0); time.sleep(0.2); self._set_high_precision()
            self.log(f"INFO: Connected to MS-2000 on {port}."); return True
        except serial.SerialException as e: self.log(f"ERROR: {e}"); self.ser=None; return False
    def disconnect(self):
        if self.is_running_scan: self.stop_scan()
        if self.ser: self.ser.close(); self.log("INFO: Disconnected.")
        self.ser = None
    def _set_high_precision(self):
        if self.ser:
            try: self.ser.write(bytes([255, 72])); time.sleep(0.1)
            except: pass
    def send_command(self, cmd):
        if not self.is_connected(): return None
        with self.lock:
            try:
                # self.log(f"CMD > {cmd}") # Раскомментировать для полной отладки
                self.ser.reset_input_buffer(); self.ser.write(f"{cmd}\r".encode('ascii'))
                return self.ser.read_until(b'\r\n').decode('ascii').strip()
            except Exception as e: self.log(f"ERROR: {e}"); return None
    def wait_for_idle(self):
        timeout = 5.0 # Адекватный таймаут на одно движение
        start_time = time.time()
        while not self.stop_event.is_set():
            if self.send_command("/") == 'N': return
            if time.time() - start_time > timeout: raise TimeoutError("Move command timed out")
            time.sleep(0.05)
    def move_absolute(self, x, y): self.send_command(f"M X={int(x*UNITS_MM_TO_DEVICE)} Y={int(y*UNITS_MM_TO_DEVICE)}")
    def run_scan(self, params, device: AcquisitionDevice, line_callback):
        self.is_running_scan = True; self.stop_event.clear()
        self.log(f"INFO: --- Starting Scan with {device} ---")
        try:
            steps_x, steps_y = int(params['steps_x']), int(params['steps_y'])
            x_coords = np.linspace(params['start_x'], params['end_x'], steps_x)
            y_coords = np.linspace(params['start_y'], params['end_y'], steps_y)
            results = np.full((steps_y, steps_x), np.nan)
            self.send_command(f"S X={params['speed']} Y={params['speed']}")
            for i, y in enumerate(y_coords):
                xs = x_coords if i % 2 == 0 else x_coords[::-1]
                for j, x in enumerate(xs):
                    if self.stop_event.is_set(): raise InterruptedError
                    self.move_absolute(x, y); self.wait_for_idle()
                    value = device.acquire(params['dwell'], x, y)
                    results[i, j if i%2==0 else (steps_x-1-j)] = value
                if line_callback: line_callback(results.copy(), i)
            self.log("INFO: --- Scan Completed ---")
        except InterruptedError: self.log("INFO: --- Scan Stopped ---"); self.send_command(chr(92))
        except Exception as e: self.log(f"ERROR: --- Scan Failed: {e} ---"); self.send_command(chr(92))
        finally: self.is_running_scan = False
    def stop_scan(self): self.log("INFO: Stop signal sent."); self.stop_event.set()

# --- 3. КЛАСС ГРАФИЧЕСКОГО ИНТЕРФЕЙСА (без изменений) ---

class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("MS-2000 Cockpit v14 (Stable)")
        self.root.geometry("700x520")
        self.controller = MS2000Controller(print)
        self.available_devices = [SmartDummySignal(), RandomNoiseDevice()]
        self.minimap_extents = [STAGE_X_MIN, STAGE_X_MAX, STAGE_Y_MIN, STAGE_Y_MAX]
        self.scan_thread=None; self.progress_line=None; self.scan_image=None; self.scan_area_patch=None
        self._pan_start_x=None; self._pan_start_y=None
        self._create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding=10); main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.columnconfigure(0, weight=1); main_frame.columnconfigure(1, weight=0); main_frame.rowconfigure(1, weight=1)
        top_left=ttk.Frame(main_frame); top_right=ttk.Frame(main_frame, width=280, height=280); bottom_left=ttk.Frame(main_frame)
        top_left.grid(row=0, column=0, sticky="nsew", padx=(0, 10)); top_right.grid(row=0, column=1, rowspan=2, sticky="ne"); bottom_left.grid(row=1, column=0, sticky="nsew", pady=(10, 0)); top_right.grid_propagate(False)
        conn_frame=ttk.LabelFrame(top_left, text="Connection", padding=10); conn_frame.pack(fill=tk.X); scan_ctrl_frame=ttk.LabelFrame(top_left, text="Scan Control", padding=10); scan_ctrl_frame.pack(fill=tk.X, pady=(10,0))
        self.conn_entries={}; ttk.Label(conn_frame, text="Port:").grid(row=0, column=0); e=ttk.Entry(conn_frame, width=8); e.insert(0, "COM4"); e.grid(row=0, column=1); self.conn_entries['port']=e
        ttk.Label(conn_frame, text="Baud:").grid(row=1, column=0); e=ttk.Entry(conn_frame, width=8); e.insert(0, "9600"); e.grid(row=1, column=1); self.conn_entries['baudrate']=e
        btn_frame=ttk.Frame(conn_frame); btn_frame.grid(row=0,column=2,rowspan=2,padx=10); self.connect_button=ttk.Button(btn_frame,text="Connect",command=self.connect); self.connect_button.pack(fill=tk.X); self.disconnect_button=ttk.Button(btn_frame,text="Disconnect",command=self.disconnect,state=tk.DISABLED); self.disconnect_button.pack(fill=tk.X, pady=2)
        ttk.Label(scan_ctrl_frame, text="Device:").pack(fill=tk.X); self.device_combobox=ttk.Combobox(scan_ctrl_frame,values=[str(d) for d in self.available_devices],state="readonly"); self.device_combobox.current(0); self.device_combobox.pack(fill=tk.X,pady=(0,5))
        self.start_scan_button=ttk.Button(scan_ctrl_frame,text="Start Scan",command=self.start_scan,state=tk.DISABLED); self.start_scan_button.pack(side=tk.LEFT,expand=True,fill=tk.X,padx=(0,5))
        self.stop_scan_button=ttk.Button(scan_ctrl_frame,text="Stop Scan",command=self.stop_scan,state=tk.DISABLED); self.stop_scan_button.pack(side=tk.LEFT,expand=True,fill=tk.X)
        self.fig=Figure(figsize=(2.8, 2.8), dpi=100); self.ax=self.fig.add_subplot(111); self.fig.subplots_adjust(left=0,right=1,top=1,bottom=0)
        self.canvas=FigureCanvasTkAgg(self.fig, master=top_right); self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True); self.setup_minimap()
        map_controls=ttk.Frame(top_right); map_controls.pack(fill=tk.X, pady=5); ttk.Label(map_controls, text="Map View:").pack(side=tk.LEFT); ttk.Button(map_controls, text="+", width=3, command=self.zoom_in).pack(side=tk.LEFT); ttk.Button(map_controls, text="-", width=3, command=self.zoom_out).pack(side=tk.LEFT); ttk.Button(map_controls, text="Reset", command=self.reset_zoom).pack(side=tk.LEFT)
        self.canvas.mpl_connect('button_press_event', self.on_pan_press); self.canvas.mpl_connect('motion_notify_event', self.on_pan_motion); self.canvas.mpl_connect('button_release_event', self.on_pan_release)
        param_frame=ttk.LabelFrame(bottom_left, text="Scan Parameters", padding=10); param_frame.pack(expand=True,fill=tk.BOTH); self.scan_entries={}
        ttk.Label(param_frame, text="X-Axis", font='-weight bold').grid(row=0, column=0, columnspan=2, pady=5); ttk.Label(param_frame, text="Y-Axis", font='-weight bold').grid(row=0, column=3, columnspan=2, pady=5)
        for i,(k,(l,v)) in enumerate({"start_x":("Start","0.0"), "steps_x":("Points","50"), "step_x":("Step","0.2")}.items()): ttk.Label(param_frame, text=l+":").grid(row=i+1,column=0,sticky="w");e=ttk.Entry(param_frame,width=8);e.insert(0,v);e.grid(row=i+1,column=1,padx=5);e.bind("<KeyRelease>", self.update_scan_area_preview);self.scan_entries[k]=e
        for i,(k,(l,v)) in enumerate({"start_y":("Start","0.0"), "steps_y":("Points","50"), "step_y":("Step","0.2")}.items()): ttk.Label(param_frame, text=l+":").grid(row=i+1,column=3,sticky="w");e=ttk.Entry(param_frame,width=8);e.insert(0,v);e.grid(row=i+1,column=4,padx=5);e.bind("<KeyRelease>", self.update_scan_area_preview);self.scan_entries[k]=e
        param_frame.columnconfigure(2,minsize=20); ttk.Separator(param_frame,orient='horizontal').grid(row=4,column=0,columnspan=5,sticky='ew',pady=10)
        for i,(k,(l,v)) in enumerate({"speed":("Speed","2.0"),"dwell":("Dwell","0.01")}.items()): ttk.Label(param_frame,text=l+":").grid(row=5,column=i*3,sticky="w");e=ttk.Entry(param_frame,width=8);e.insert(0,v);e.grid(row=5,column=i*3+1,padx=5);self.scan_entries[k]=e
    
    def setup_minimap(self):
        self.ax.clear(); self.ax.set_xticks([]); self.ax.set_yticks([]); self.ax.set_facecolor('#cccccc'); self.ax.set_aspect('equal', adjustable='box')
        self.scan_image = None; self.scan_area_patch = None
        if self.progress_line: self.progress_line.remove(); self.progress_line = None
        self.update_minimap_view()
    def update_minimap_view(self): self.ax.set_xlim(self.minimap_extents[0:2]); self.ax.set_ylim(self.minimap_extents[2:4]); self.canvas.draw()
    def zoom_in(self): x0,x1,y0,y1=self.minimap_extents;cx,cy=(x0+x1)/2,(y0+y1)/2;w,h=(x1-x0)/4,(y1-y0)/4;self.minimap_extents=[cx-w,cx+w,cy-h,cy+h];self.update_minimap_view()
    def zoom_out(self): x0,x1,y0,y1=self.minimap_extents;cx,cy=(x0+x1)/2,(y0+y1)/2;w,h=(x1-x0),(y1-y0);self.minimap_extents=[max(STAGE_X_MIN,cx-w),min(STAGE_X_MAX,cx+w),max(STAGE_Y_MIN,cy-h),min(STAGE_Y_MAX,cy+h)];self.update_minimap_view()
    def reset_zoom(self): self.minimap_extents = [STAGE_X_MIN, STAGE_X_MAX, STAGE_Y_MIN, STAGE_Y_MAX]; self.update_minimap_view()
    def on_pan_press(self, e):
        if e.inaxes != self.ax: return
        self._pan_start_x, self._pan_start_y = e.xdata, e.ydata
    def on_pan_release(self, e): self._pan_start_x, self._pan_start_y = None, None
    def on_pan_motion(self, e):
        if self._pan_start_x is None or e.inaxes != self.ax: return
        dx = e.xdata - self._pan_start_x; dy = e.ydata - self._pan_start_y
        self.minimap_extents[0]-=dx; self.minimap_extents[1]-=dx; self.minimap_extents[2]-=dy; self.minimap_extents[3]-=dy
        self.update_minimap_view()
    def update_scan_area_preview(self, e=None):
        if self.scan_area_patch: self.scan_area_patch.remove()
        try:
            p=self.get_scan_params(False); w=(p['steps_x']-1)*p['step_x']; h=(p['steps_y']-1)*p['step_y']
            self.scan_area_patch = self.ax.add_patch(Rectangle((p['start_x'],p['start_y']),w,h,lw=1,ec='black',fc='black',alpha=0.5)); self.canvas.draw()
        except: pass
    def auto_zoom_to_scan_area(self, params):
        min_x=min(params['start_x'],params['end_x']); max_x=max(params['start_x'],params['end_x']); min_y=min(params['start_y'],params['end_y']); max_y=max(params['start_y'],params['end_y'])
        width=max_x-min_x; height=max_y-min_y; margin_x=max(width*0.1,0.5); margin_y=max(height*0.1,0.5)
        self.minimap_extents = [min_x-margin_x, max_x+margin_x, min_y-margin_y, max_y+margin_y]; self.update_minimap_view()
    def start_scan(self):
        params=self.get_scan_params(); dev_str=self.device_combobox.get()
        if params is None or not dev_str: return
        device = next((d for d in self.available_devices if str(d) == dev_str), None)
        self.start_scan_button.config(state=tk.DISABLED); self.stop_scan_button.config(state=tk.NORMAL)
        self.auto_zoom_to_scan_area(params)
        nan_data = np.full((int(params['steps_y']), int(params['steps_x'])), np.nan); self.plot_scan_data(nan_data, -1) # Начинаем с -1, чтобы линия была внизу
        self.scan_thread = threading.Thread(target=self.controller.run_scan, args=(params, device, self.line_update_callback), daemon=True); self.scan_thread.start()
        self.check_scan_thread()
    def line_update_callback(self, data, row_idx): self.root.after(0, self.plot_scan_data, data, row_idx)
    def plot_scan_data(self, data, row_index):
        p=self.get_scan_params(False);
        if not p: return
        extent=[p['start_x'],p['end_x'],p['start_y'],p['end_y']]
        cmap=cm.get_cmap('viridis').copy();cmap.set_bad(color='black')
        if not self.scan_image: self.scan_image = self.ax.imshow(data, cmap=cmap, origin='lower', extent=extent, interpolation='none', vmin=0, vmax=100)
        else: self.scan_image.set_data(data); self.scan_image.set_extent(extent)
        if not np.all(np.isnan(data)): self.scan_image.set_clim(np.nanmin(data), np.nanmax(data))
        if self.progress_line: self.progress_line.remove()
        
        step_y = (p['end_y']-p['start_y'])/(p['steps_y']-1) if p['steps_y']>1 else 0
        line_y_pos = p['start_y'] + (row_index + 1) * step_y - (step_y / 2)
        if row_index < int(p['steps_y'] -1):
            self.progress_line = self.ax.axhline(y=line_y_pos, color='yellow', lw=2, alpha=0.9)
        else:
            self.progress_line = None # Линия исчезает после последней строки
            
        self.canvas.draw()
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected(): self.start_scan_button.config(state=tk.NORMAL)
            self.reset_zoom(); self.setup_minimap(); self.update_scan_area_preview()
    def get_scan_params(self, validate=True):
        try:
            p={k:float(e.get()) for k,e in self.scan_entries.items()}
            if validate:
                if (p['steps_x']%1!=0 or p['steps_y']%1!=0 or p['steps_x']<1 or p['steps_y']<1):
                    messagebox.showerror("Parameter Error", "Points must be integers >= 1."); return None
            p['end_x']=p['start_x']+(p['steps_x']-1)*p['step_x']; p['end_y']=p['start_y']+(p['steps_y']-1)*p['step_y']
            return p
        except (ValueError, tk.TclError):
            if validate: messagebox.showerror("Parameter Error", "All fields must contain valid numbers.");
            return None
    def connect(self):
        if self.controller.connect(self.conn_entries['port'].get(), int(self.conn_entries['baudrate'].get())):
            self.connect_button.config(state=tk.DISABLED);self.disconnect_button.config(state=tk.NORMAL);self.start_scan_button.config(state=tk.NORMAL)
    def disconnect(self): self.controller.disconnect();self.connect_button.config(state=tk.NORMAL);self.disconnect_button.config(state=tk.DISABLED);self.start_scan_button.config(state=tk.DISABLED);self.stop_scan_button.config(state=tk.DISABLED)
    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive(): self.controller.stop_scan()
    def on_closing(self): self.controller.disconnect(); self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()