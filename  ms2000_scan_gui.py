# ms2000_cockpit.py
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import time
import threading
import numpy as np
from typing import Optional, Callable, Any
import abc
import os
from datetime import datetime

try:
    from scipy.optimize import curve_fit
except ImportError:
    pass 

import timetagger_device

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import matplotlib.cm as cm
from matplotlib.patches import Rectangle

UNITS_UM_TO_DEVICE = 10; STAGE_X_MIN, STAGE_X_MAX = -34000.0, 39000.0; STAGE_Y_MIN, STAGE_Y_MAX = -34000.0, 39000.0

class AcquisitionDevice(abc.ABC):
    def setup_scan(self, total_pixels: int, dwell_s: float, mode: str): pass
    @abc.abstractmethod
    def acquire(self, dwell_time: float, x: float, y: float) -> Any: pass
    def teardown_scan(self): pass
    def close(self): pass
    def __str__(self): return self.__class__.__name__

class SmartDummySignal(AcquisitionDevice):
    def setup_scan(self, total_pixels: int, dwell_s: float, mode: str):
        self.is_flim = (mode == "FLIM")
    def acquire(self, dwell_time: float, x: float, y: float) -> Any:
        time.sleep(dwell_time)
        center_x, center_y, radius = 10000.0, 10000.0, 5000.0
        dist = np.sqrt((x-center_x)**2 + (y-center_y)**2)
        val = 90 + np.random.rand()*10 if dist < radius else 10 + np.random.rand()*10
        if self.is_flim:
            return np.random.poisson(lam=val/10.0, size=1000)
        return val

class MS2000Controller:
    def __init__(self, log_callback: Callable[[str], None]):
        self.ser: Optional[serial.Serial] = None; self.is_running_scan = False
        self.stop_event = threading.Event(); self.log = log_callback; self.lock = threading.Lock()
        self.invert_x = tk.BooleanVar(value=False); self.invert_y = tk.BooleanVar(value=True); self.swap_xy = tk.BooleanVar(value=True)
    
    def _transform_coords_to_stage(self, x_gui, y_gui):
        x_stage, y_stage = (y_gui, x_gui) if self.swap_xy.get() else (x_gui, y_gui)
        if self.invert_x.get(): x_stage *= -1
        if self.invert_y.get(): y_stage *= -1
        return x_stage, y_stage
    
    def _transform_coords_from_stage(self, x_stage, y_stage):
        x_temp, y_temp = x_stage, y_stage
        if self.invert_x.get(): x_temp *= -1
        if self.invert_y.get(): y_temp *= -1
        return (y_temp, x_temp) if self.swap_xy.get() else (x_temp, y_temp)
    
    def is_connected(self) -> bool: return self.ser is not None and self.ser.is_open
    
    def connect(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0); time.sleep(0.05); self._set_high_precision()
            self.log(f"INFO: Connected to MS-2000 on {port}.")
            self.send_command("B X=0 Y=0", quiet=True)
            return True
        except serial.SerialException as e: self.log(f"ERROR: {e}"); self.ser=None; return False
        
    def disconnect(self):
        if self.is_running_scan: self.stop_scan()
        if self.ser: self.ser.close(); self.log("INFO: Disconnected.")
        self.ser = None
        
    def _set_high_precision(self):
        if self.is_connected():
            try: self.ser.write(bytes([255, 72])); time.sleep(0.05)
            except: pass
            
    def send_command(self, cmd, quiet=False):
        if not self.is_connected(): return None
        with self.lock:
            try:
                self.ser.reset_input_buffer(); self.ser.write(f"{cmd}\r".encode('ascii'))
                return self.ser.read_until(b'\r\n').decode('ascii').strip()
            except Exception as e: self.log(f"ERROR: {e}"); return None
            
    def wait_for_idle(self):
        timeout = 15.0; start_time = time.time()
        while not self.stop_event.is_set():
            if self.send_command("/", quiet=True) == 'N': return
            if time.time() - start_time > timeout: raise TimeoutError("Move command timed out")
            time.sleep(0.02)
            
    def get_position(self) -> Optional[tuple[float, float, float]]:
        response = self.send_command("W X Y Z", quiet=True)
        if response and response.startswith(":A"):
            try:
                parts = response.split()
                x_stage = float(parts[1])/UNITS_UM_TO_DEVICE; y_stage = float(parts[2])/UNITS_UM_TO_DEVICE; z_stage = float(parts[3])/UNITS_UM_TO_DEVICE
                x_gui, y_gui = self._transform_coords_from_stage(x_stage, y_stage)
                return (x_gui, y_gui, z_stage)
            except: self.log("ERROR: Could not parse position."); return None
        return None
        
    def move_absolute(self, x_gui, y_gui):
        x_stage, y_stage = self._transform_coords_to_stage(x_gui, y_gui)
        self.send_command(f"M X={int(x_stage*UNITS_UM_TO_DEVICE)} Y={int(y_stage*UNITS_UM_TO_DEVICE)}")
        
    def move_absolute_z(self, z_gui):
        self.send_command(f"M Z={int(z_gui*UNITS_UM_TO_DEVICE)}")
        
    def move_relative(self, dx_gui, dy_gui):
        dx_stage, dy_stage = dx_gui, dy_gui
        if self.swap_xy.get(): dx_stage, dy_stage = dy_gui, dx_gui
        if self.invert_x.get(): dx_stage *= -1
        if self.invert_y.get(): dy_stage *= -1
        cmd = "R"
        if int(dx_stage*UNITS_UM_TO_DEVICE) != 0: cmd += f" X={int(dx_stage*UNITS_UM_TO_DEVICE)}"
        if int(dy_stage*UNITS_UM_TO_DEVICE) != 0: cmd += f" Y={int(dy_stage*UNITS_UM_TO_DEVICE)}"
        if cmd != "R": self.send_command(cmd)
        
    def move_relative_z(self, dz_gui):
        self.send_command(f"R Z={int(dz_gui*UNITS_UM_TO_DEVICE)}")

    def run_scan(self, params, device: AcquisitionDevice, line_callback):
        self.is_running_scan = True; self.stop_event.clear()
        self.log(f"INFO: --- Starting Raster Scan ({params['mode']}) ---")
        
        center_x = (params['start_x'] + params['end_x']) / 2.0
        center_y = (params['start_y'] + params['end_y']) / 2.0
        travel_speed = 0.5
        
        try:
            scan_speed = params['speed'] / 1000.0; backlash = params['backlash']
            steps_x, steps_y = int(params['steps_x']), int(params['steps_y'])
            x_coords = np.linspace(params['start_x'], params['end_x'], steps_x)
            y_coords = np.linspace(params['start_y'], params['end_y'], steps_y)
            
            if params['mode'] == "FLIM":
                n_bins = getattr(device, 'flim_n_bins', 1000) 
                results = np.full((steps_y, steps_x, n_bins), np.nan)
            else:
                results = np.full((steps_y, steps_x), np.nan)
            
            device.setup_scan(steps_x * steps_y, params['acc_time'], params['mode'])
            
            self.send_command(f"S X={travel_speed} Y={travel_speed}", quiet=True)
            self.move_absolute(x_coords[0] - backlash, y_coords[0] - backlash); self.wait_for_idle()
            self.move_absolute(x_coords[0], y_coords[0]); self.wait_for_idle()
            
            self.send_command(f"S X={scan_speed} Y={scan_speed}")

            for i, y in enumerate(y_coords):
                if i > 0:
                    self.send_command(f"S X={travel_speed} Y={travel_speed}", quiet=True)
                    self.move_absolute(x_coords[0] - backlash, y - backlash); self.wait_for_idle()
                    self.move_absolute(x_coords[0], y); self.wait_for_idle()
                    self.send_command(f"S X={scan_speed} Y={scan_speed}", quiet=True)
                
                for j, x in enumerate(x_coords):
                    if self.stop_event.is_set(): raise InterruptedError
                    if j > 0:
                        self.move_absolute(x, y); self.wait_for_idle()

                    self.send_command("TTL Y=1", quiet=True); self.send_command("TTL Y=0", quiet=True)
                    value = device.acquire(params['acc_time'], x, y)
                    results[i, j] = value
                
                if line_callback: line_callback(results.copy(), i)
                
        except InterruptedError: 
            self.log("INFO: --- Scan Aborted ---")
            self.send_command(chr(92))
        except Exception as e: 
            self.log(f"ERROR: --- Scan Failed: {e} ---")
            self.send_command(chr(92))
        finally: 
            # ВСЕГДА возвращаемся в центр
            self.log("INFO: Returning to Center position...")
            self.send_command(f"S X={travel_speed} Y={travel_speed}", quiet=True)
            self.move_absolute(center_x, center_y); self.wait_for_idle()
            device.teardown_scan()
            self.is_running_scan = False
            
    def stop_scan(self): self.log("INFO: Stop signal sent."); self.stop_event.set()


class StageControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root; self.root.title("MS-2000 Cockpit v37 (Pro Edition)"); self.root.geometry("850x650")
        self.controller = MS2000Controller(lambda msg: print(f"{time.strftime('%H:%M:%S')} - {msg}"))
        
        self.tt_device_instance = None
        self.active_device = None
        
        self.minimap_extents =[STAGE_X_MIN, STAGE_X_MAX, STAGE_Y_MIN, STAGE_Y_MAX]
        self.scan_thread=None; self.progress_line=None; self.scan_image=None; self.scan_area_patch=None
        self.cbar = None
        self.raw_scan_data = None
        self.tau_map = None 
        
        self._pan_start_x=None; self._pan_start_y=None
        self._position_updater_job = None
        self.start_mode_var = tk.StringVar(value="center")
        
        self._create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding=10); main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.columnconfigure(0, weight=1); main_frame.columnconfigure(1, weight=0); main_frame.rowconfigure(2, weight=1)
        top_left=ttk.Frame(main_frame); top_right=ttk.Frame(main_frame, width=320)
        bottom_left=ttk.Frame(main_frame)
        top_left.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 10)); top_right.grid(row=0, column=1, rowspan=3, sticky="ns"); bottom_left.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        
        conn_frame=ttk.LabelFrame(top_left, text="Connection", padding=10); conn_frame.pack(fill=tk.X)
        self.conn_entries={}; ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w"); e=ttk.Entry(conn_frame, width=8); e.insert(0, "COM4"); e.grid(row=0, column=1); self.conn_entries['port']=e
        ttk.Label(conn_frame, text="Baud:").grid(row=1, column=0, sticky="w"); e=ttk.Entry(conn_frame, width=8); e.insert(0, "9600"); e.grid(row=1, column=1); self.conn_entries['baudrate']=e
        check_frame = ttk.Frame(conn_frame); check_frame.grid(row=0, column=2, rowspan=2, padx=10)
        ttk.Checkbutton(check_frame, text="Swap XY", variable=self.controller.swap_xy).pack(anchor='w')
        ttk.Checkbutton(check_frame, text="Invert X", variable=self.controller.invert_x).pack(anchor='w')
        ttk.Checkbutton(check_frame, text="Invert Y", variable=self.controller.invert_y).pack(anchor='w')
        btn_frame=ttk.Frame(conn_frame); btn_frame.grid(row=0,column=3,rowspan=2,padx=5)
        self.connect_button=ttk.Button(btn_frame,text="Connect",command=self.connect); self.connect_button.pack(fill=tk.X)
        self.disconnect_button=ttk.Button(btn_frame,text="Disconnect",command=self.disconnect,state=tk.DISABLED); self.disconnect_button.pack(fill=tk.X, pady=2)
        
        scan_ctrl_frame=ttk.LabelFrame(top_left, text="Scan Control", padding=10); scan_ctrl_frame.pack(fill=tk.X, pady=(10,0))
        dev_frame = ttk.Frame(scan_ctrl_frame); dev_frame.pack(fill=tk.X, pady=(0,5))
        ttk.Label(dev_frame, text="Device:").pack(side=tk.LEFT)
        self.device_combobox=ttk.Combobox(dev_frame, values=["SmartDummySignal", "TimeTagger"], state="readonly", width=15)
        self.device_combobox.current(0); self.device_combobox.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        self.btn_settings = ttk.Button(dev_frame, text="⚙️", width=3, command=self.open_device_settings)
        self.btn_settings.pack(side=tk.LEFT)
        
        mode_frame = ttk.Frame(scan_ctrl_frame); mode_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(mode_frame, text="Mode:").grid(row=0, column=0, sticky='w')
        self.scan_mode_cb = ttk.Combobox(mode_frame, values=["Intensity", "FLIM"], state="readonly", width=9)
        self.scan_mode_cb.current(0); self.scan_mode_cb.grid(row=0, column=1, padx=2)
        
        ttk.Label(mode_frame, text="FLIM Fit:").grid(row=0, column=2, sticky='w', padx=(5,0))
        self.flim_calc_cb = ttk.Combobox(mode_frame, values=["Fast FLIM", "Scipy Fit"], state="readonly", width=9)
        self.flim_calc_cb.current(0); self.flim_calc_cb.grid(row=0, column=3, padx=2)
        
        ttk.Label(mode_frame, text="Min Photons:").grid(row=1, column=0, columnspan=2, sticky='w', pady=(5,0))
        self.ent_min_photons = ttk.Entry(mode_frame, width=9)
        self.ent_min_photons.insert(0, "50")
        self.ent_min_photons.grid(row=1, column=2, columnspan=2, sticky='w', pady=(5,0), padx=2)
        
        start_mode_frame = ttk.Frame(scan_ctrl_frame); start_mode_frame.pack(fill=tk.X, pady=(10, 5))
        ttk.Radiobutton(start_mode_frame, text="Auto-Center at Current Pos", variable=self.start_mode_var, value="center").pack(anchor='w')
        ttk.Radiobutton(start_mode_frame, text="Start from Current Pos", variable=self.start_mode_var, value="start").pack(anchor='w')
        ttk.Radiobutton(start_mode_frame, text="Use Manual Start Coords", variable=self.start_mode_var, value="manual").pack(anchor='w')

        btn_row_1 = ttk.Frame(scan_ctrl_frame); btn_row_1.pack(fill=tk.X)
        self.start_scan_button=ttk.Button(btn_row_1,text="Start Scan",command=self.start_scan,state=tk.DISABLED); self.start_scan_button.pack(side=tk.LEFT,expand=True,fill=tk.X,padx=(0,2))
        self.stop_scan_button=ttk.Button(btn_row_1,text="Stop Scan",command=self.stop_scan,state=tk.DISABLED); self.stop_scan_button.pack(side=tk.LEFT,expand=True,fill=tk.X,padx=(2,0))
        
        self.fig=Figure(figsize=(3.2, 3.2), dpi=100); self.ax=self.fig.add_subplot(111); self.fig.subplots_adjust(left=0.15,right=0.85,top=0.9,bottom=0.15)
        self.canvas=FigureCanvasTkAgg(self.fig, master=top_right); self.canvas.get_tk_widget().pack(fill=tk.X, anchor='n'); self.setup_minimap()
        
        self.progress_bar = ttk.Progressbar(top_right, orient="horizontal", mode="determinate")
        self.progress_bar.pack(fill=tk.X, pady=5)
        
        toolbar_frame = ttk.Frame(top_right); toolbar_frame.pack(fill=tk.X, anchor='n')
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame); self.toolbar.update()
        
        manual_frame = ttk.LabelFrame(top_right, text="Manual Control", padding=10); manual_frame.pack(fill=tk.X, pady=(10,0), anchor='n')
        pos_frame = ttk.Frame(manual_frame); pos_frame.pack(fill=tk.X, pady=(0,10))
        ttk.Label(pos_frame, text="X Pos:").grid(row=0, column=0, sticky='w'); self.x_pos_entry = ttk.Entry(pos_frame, width=8, state='readonly'); self.x_pos_entry.grid(row=0, column=1)
        ttk.Label(pos_frame, text="Y Pos:").grid(row=1, column=0, sticky='w'); self.y_pos_entry = ttk.Entry(pos_frame, width=8, state='readonly'); self.y_pos_entry.grid(row=1, column=1)
        ttk.Label(pos_frame, text="Z Pos:").grid(row=2, column=0, sticky='w'); self.z_pos_entry = ttk.Entry(pos_frame, width=8, state='readonly'); self.z_pos_entry.grid(row=2, column=1)
        jog_frame = ttk.Frame(manual_frame); jog_frame.pack()
        ttk.Button(jog_frame, text="↑", command=lambda: self._manual_move(0, -1)).grid(row=0, column=1)
        ttk.Button(jog_frame, text="←", command=lambda: self._manual_move(-1, 0)).grid(row=1, column=0)
        ttk.Button(jog_frame, text="→", command=lambda: self._manual_move(1, 0)).grid(row=1, column=2)
        ttk.Button(jog_frame, text="↓", command=lambda: self._manual_move(0, 1)).grid(row=2, column=1)
        ttk.Separator(jog_frame, orient='vertical').grid(row=0, column=3, rowspan=3, sticky='ns', padx=10)
        ttk.Button(jog_frame, text="↑ Z", command=lambda: self._manual_move_z(1)).grid(row=0, column=4, rowspan=2, sticky='ns')
        ttk.Button(jog_frame, text="↓ Z", command=lambda: self._manual_move_z(-1)).grid(row=2, column=4, rowspan=1, sticky='sew')
        z_step_frame = ttk.Frame(manual_frame); z_step_frame.pack(fill=tk.X, pady=(10,0))
        ttk.Label(z_step_frame, text="Z Step (um):").pack(side=tk.LEFT)
        self.z_step_entry = ttk.Entry(z_step_frame, width=6); self.z_step_entry.insert(0, "0.5"); self.z_step_entry.pack(side=tk.LEFT)

        param_frame=ttk.LabelFrame(bottom_left, text="Scan Parameters", padding=10); param_frame.pack(expand=True,fill=tk.BOTH)
        self.scan_entries={}
        ttk.Label(param_frame, text="X-Axis", font='-weight bold').grid(row=0, column=0, columnspan=3, pady=5)
        ttk.Label(param_frame, text="Y-Axis", font='-weight bold').grid(row=0, column=3, columnspan=3, pady=5)
        scan_fields_x = {"start_x":("Start", "0.0", "um"), "steps_x":("Points","20", ""), "step_x":("Step","1.0", "um")}
        scan_fields_y = {"start_y":("Start","0.0", "um"), "steps_y":("Points","20", ""), "step_y":("Step","1.0", "um")}
        
        for i,(k,(l,v,u)) in enumerate(scan_fields_x.items()): 
            ttk.Label(param_frame, text=l+":").grid(row=i+1,column=0,sticky="w"); e=ttk.Entry(param_frame,width=8); e.insert(0,v); e.grid(row=i+1,column=1,padx=5); e.bind("<KeyRelease>", self.update_scan_area_preview); self.scan_entries[k]=e; ttk.Label(param_frame, text=u).grid(row=i+1, column=2, sticky='w')
        for i,(k,(l,v,u)) in enumerate(scan_fields_y.items()): 
            ttk.Label(param_frame, text=l+":").grid(row=i+1,column=3,sticky="w"); e=ttk.Entry(param_frame,width=8); e.insert(0,v); e.grid(row=i+1,column=4,padx=5); e.bind("<KeyRelease>", self.update_scan_area_preview); self.scan_entries[k]=e; ttk.Label(param_frame, text=u).grid(row=i+1, column=5, sticky='w')
            
        param_frame.columnconfigure(2,minsize=15); param_frame.columnconfigure(5,minsize=15)
        ttk.Separator(param_frame,orient='horizontal').grid(row=4,column=0,columnspan=6,sticky='ew',pady=10)
        general_fields = { "acc_time":("Acc. Time","1.0", "s"), "speed":("Speed","2000.0", "um/s"), "backlash":("Backlash", "1.0", "um") }
        start_row = 5
        for i, (k, (l, v, u)) in enumerate(general_fields.items()):
            ttk.Label(param_frame, text=l+":").grid(row=start_row + i, column=0, sticky="w", pady=2); e=ttk.Entry(param_frame, width=8); e.insert(0,v); e.grid(row=start_row + i, column=1, padx=5, pady=2); self.scan_entries[k]=e; ttk.Label(param_frame, text=u).grid(row=start_row + i, column=2, sticky="w", pady=2)

        next_row = start_row + len(general_fields)
        ttk.Separator(param_frame, orient='horizontal').grid(row=next_row, column=0, columnspan=6, sticky='ew', pady=5)
        ttk.Label(param_frame, text="File Name:", font='-weight bold').grid(row=next_row+1, column=0, sticky="w", pady=2)
        self.entry_filename = ttk.Entry(param_frame, width=15); self.entry_filename.insert(0, "sample")
        self.entry_filename.grid(row=next_row+1, column=1, columnspan=2, sticky="w", padx=5, pady=2)

    def open_device_settings(self):
        dev = self.device_combobox.get()
        if dev == "TimeTagger":
            if not self.tt_device_instance: self.tt_device_instance = timetagger_device.TimeTaggerDevice()
            if not self.tt_device_instance.is_connected: messagebox.showwarning("Внимание", "Time Tagger не обнаружен.", parent=self.root)
            timetagger_device.TTSettingsPopup(self.root, self.tt_device_instance)
        else:
            messagebox.showinfo("Settings", f"Дополнительные настройки для {dev} не требуются.", parent=self.root)

    def _manual_move(self, dx_factor, dy_factor):
        if not self.controller.is_connected(): messagebox.showwarning("Warning", "Not connected."); return
        try:
            step_x = float(self.scan_entries['step_x'].get()); step_y = float(self.scan_entries['step_y'].get())
            speed = float(self.scan_entries['speed'].get()) / 1000.0
            dx = step_x * dx_factor; dy = step_y * dy_factor
            def move_with_speed():
                self.controller.send_command(f"S X={speed} Y={speed}", quiet=True)
                self.controller.move_relative(dx, dy)
            threading.Thread(target=move_with_speed, daemon=True).start()
        except (ValueError, KeyError): messagebox.showerror("Error", "Invalid step or speed value.")

    def _manual_move_z(self, dz_factor):
        if not self.controller.is_connected(): messagebox.showwarning("Warning", "Not connected."); return
        try:
            step_z = float(self.z_step_entry.get()); speed = float(self.scan_entries['speed'].get()) / 1000.0
            dz = step_z * dz_factor
            def move_with_speed():
                self.controller.send_command(f"S Z={speed}", quiet=True)
                self.controller.move_relative_z(dz)
            threading.Thread(target=move_with_speed, daemon=True).start()
        except (ValueError, KeyError): messagebox.showerror("Error", "Invalid Z-step or speed value.")
        
    def set_current_position_as_center(self) -> bool:
        if not self.controller.is_connected(): messagebox.showwarning("Warning", "Not connected."); return False
        params = self.get_scan_params(validate=False)
        if not params: messagebox.showerror("Error", "Invalid scan size parameters."); return False
        pos = self.controller.get_position()
        if pos:
            center_x, center_y, _ = pos
            total_width=(params['steps_x']-1)*params['step_x']; total_height=(params['steps_y']-1)*params['step_y']
            new_start_x=center_x-total_width/2.0; new_start_y=center_y-total_height/2.0
            self.scan_entries['start_x'].delete(0,tk.END); self.scan_entries['start_x'].insert(0,f"{new_start_x:.1f}")
            self.scan_entries['start_y'].delete(0,tk.END); self.scan_entries['start_y'].insert(0,f"{new_start_y:.1f}")
            self.update_scan_area_preview()
            return True
        return False
        
    def set_current_position_as_start(self) -> bool:
        if not self.controller.is_connected(): messagebox.showwarning("Warning", "Not connected."); return False
        pos = self.controller.get_position()
        if pos:
            x, y, _ = pos
            self.scan_entries['start_x'].delete(0, tk.END); self.scan_entries['start_x'].insert(0, f"{x:.1f}")
            self.scan_entries['start_y'].delete(0, tk.END); self.scan_entries['start_y'].insert(0, f"{y:.1f}")
            self.update_scan_area_preview()
            return True
        return False

    def _update_position_display(self):
        if self.controller.is_connected():
            pos = self.controller.get_position()
            if pos:
                x, y, z = pos
                self.x_pos_entry.config(state='normal'); self.x_pos_entry.delete(0, tk.END); self.x_pos_entry.insert(0, f"{x:.2f}"); self.x_pos_entry.config(state='readonly')
                self.y_pos_entry.config(state='normal'); self.y_pos_entry.delete(0, tk.END); self.y_pos_entry.insert(0, f"{y:.2f}"); self.y_pos_entry.config(state='readonly')
                self.z_pos_entry.config(state='normal'); self.z_pos_entry.delete(0, tk.END); self.z_pos_entry.insert(0, f"{z:.2f}"); self.z_pos_entry.config(state='readonly')
        self._position_updater_job = self.root.after(250, self._update_position_display)

    def _start_position_updater(self): self._update_position_display()
    def _stop_position_updater(self):
        if self._position_updater_job: self.root.after_cancel(self._position_updater_job); self._position_updater_job = None
        
    def setup_minimap(self):self.ax.clear();self.ax.set_facecolor('#cccccc');self.scan_image=None;self.scan_area_patch=None;self.update_minimap_view()
    def update_minimap_view(self): self.canvas.draw()
    def zoom_in(self): pass
    def zoom_out(self): pass
    def reset_zoom(self): pass
    def on_pan_press(self,e): pass
    def on_pan_release(self,e): pass
    def on_pan_motion(self,e): pass
        
    def update_scan_area_preview(self,e=None):
        if self.scan_area_patch:
            try: self.scan_area_patch.remove()
            except ValueError: pass
            self.scan_area_patch = None
        try: p=self.get_scan_params(False);w=(p['steps_x']-1)*p['step_x'];h=(p['steps_y']-1)*p['step_y']; self.scan_area_patch=self.ax.add_patch(Rectangle((p['start_x'],p['start_y']),w,h,lw=1,ec='black',fc='black',alpha=0.5));self.canvas.draw()
        except: pass
        
    def start_scan(self):
        start_mode = self.start_mode_var.get()
        if start_mode == "center":
            if not self.set_current_position_as_center(): return
        elif start_mode == "start":
            if not self.set_current_position_as_start(): return
        
        params=self.get_scan_params(); dev_str=self.device_combobox.get()
        if params is None or not dev_str: return
        params['mode'] = self.scan_mode_cb.get()
        
        if dev_str == "TimeTagger":
            if not self.tt_device_instance: self.tt_device_instance = timetagger_device.TimeTaggerDevice()
            self.active_device = self.tt_device_instance
        else: self.active_device = SmartDummySignal()
            
        self.start_scan_button.config(state=tk.DISABLED);self.stop_scan_button.config(state=tk.NORMAL)
        self.progress_bar['value'] = 0
        
        self.tau_map = np.full((int(params['steps_y']), int(params['steps_x'])), np.nan)
        
        self.ax.set_xlim(params['start_x'], params['end_x'])
        self.ax.set_ylim(params['start_y'], params['end_y'])
        
        self.scan_thread=threading.Thread(target=self.controller.run_scan,args=(params,self.active_device,self.line_update_callback),daemon=True);self.scan_thread.start()
        self.check_scan_thread()
        
    def line_update_callback(self,data,row_idx): self.root.after(0,self.plot_scan_data,data,row_idx)
    
    def plot_scan_data(self, data, row_index):
        p = self.get_scan_params(False)
        if not p: return
        extent = [p['start_x'], p['end_x'], p['start_y'], p['end_y']]
        self.raw_scan_data = data.copy()
        
        pct = (row_index + 1) / p['steps_y'] * 100
        self.progress_bar['value'] = pct
        
        try:
            min_photons = int(self.ent_min_photons.get())
        except ValueError:
            min_photons = 50
        
        if data.ndim == 3:
            bin_ps = getattr(self.active_device, 'flim_binwidth_ps', 1000)
            calc_method = self.flim_calc_cb.get()

            global_hist = np.nansum(data, axis=(0, 1))
            peak_idx = int(np.argmax(global_hist)) if np.sum(global_hist) > 0 else 0
            
            if row_index >= 0:
                for x_idx in range(data.shape[1]):
                    pixel_data = data[row_index, x_idx, :]
                    
                    if np.isnan(pixel_data).all() or np.sum(pixel_data) < min_photons:
                        self.tau_map[row_index, x_idx] = np.nan
                        continue
                    
                    if peak_idx > 5: baseline = np.nanmean(pixel_data[:peak_idx-2])
                    else: baseline = 0
                        
                    decay = pixel_data[peak_idx:] - baseline
                    decay[decay < 0] = 0
                    
                    sum_I = np.sum(decay)
                    if sum_I == 0:
                        self.tau_map[row_index, x_idx] = np.nan
                        continue
                        
                    T = np.arange(len(decay)) * (bin_ps / 1000.0)
                    fast_tau = np.sum(decay * T) / sum_I
                    
                    if calc_method == "Scipy Fit":
                        try:
                            popt, _ = curve_fit(
                                lambda t, a, tau, c: a * np.exp(-t / tau) + c,
                                T, decay,
                                p0=[np.max(decay), fast_tau, 0],
                                bounds=(0, np.inf),
                                maxfev=400
                            )
                            self.tau_map[row_index, x_idx] = popt[1]
                        except Exception:
                            self.tau_map[row_index, x_idx] = fast_tau
                    else:
                        self.tau_map[row_index, x_idx] = fast_tau
                        
            plot_data = self.tau_map
            cbar_label = "Lifetime (ns)"
        else:
            plot_data = data
            plot_data[plot_data < min_photons] = np.nan
            cbar_label = "Photon Counts"

        cmap = cm.get_cmap('viridis').copy(); cmap.set_bad(color='#222222')
        
        if self.cbar:
            self.cbar.remove()
            
        if not self.scan_image:
            self.scan_image=self.ax.imshow(plot_data,cmap=cmap,origin='lower',extent=extent,interpolation='none')
        else:
            self.scan_image.set_data(plot_data)
            self.scan_image.set_extent(extent)
            
        self.cbar = self.fig.colorbar(self.scan_image, ax=self.ax)
        self.cbar.set_label(cbar_label)
        
        valid_data = plot_data[~np.isnan(plot_data)]
        if len(valid_data) > 0:
            vmax = np.percentile(valid_data, 98.0)
            vmin = np.min(valid_data)
            if vmax <= vmin: vmax = vmin + 0.1
            self.scan_image.set_clim(vmin, vmax)
            
        if self.progress_line:self.progress_line.remove()
        step_y=(p['end_y']-p['start_y'])/(p['steps_y']-1) if p['steps_y']>1 else 0; line_y_pos=p['start_y']+(row_index+0.5)*step_y
        if row_index<int(p['steps_y']-1):self.progress_line=self.ax.axhline(y=line_y_pos,color='yellow',lw=2,alpha=0.9)
        else:self.progress_line=None
        self.canvas.draw()
        
    def check_scan_thread(self):
        if self.scan_thread and self.scan_thread.is_alive():
            self.root.after(100, self.check_scan_thread)
        else:
            self.stop_scan_button.config(state=tk.DISABLED)
            if self.controller.is_connected():
                self.start_scan_button.config(state=tk.NORMAL)
            
            if hasattr(self, 'raw_scan_data') and self.raw_scan_data is not None and self.active_device is not None:
                self.save_scan_results()
            
            # self.reset_zoom(); self.setup_minimap(); self.update_scan_area_preview()

    def save_scan_results(self):
        os.makedirs("results", exist_ok=True)
        base_name = self.entry_filename.get().strip() or "scan"
        p = self.get_scan_params(validate=False)
        mode = self.scan_mode_cb.get()
        params_str = f"{int(p['steps_x'])}x{int(p['steps_y'])}_{p['step_x']}um_{p['acc_time']}s_{mode}" if p else "unknown"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_prefix = f"{base_name}_{params_str}_{timestamp}"
        
        if self.raw_scan_data.ndim == 3:
            npy_filename = f"results/{file_prefix}.npy"
            np.save(npy_filename, self.raw_scan_data)
            self.controller.log(f"INFO: 3D FLIM Data saved to {npy_filename}")
            
            intensity_2d = np.sum(self.raw_scan_data, axis=2)
            csv_intensity_file = f"results/{file_prefix}_intensity.csv"
            np.savetxt(csv_intensity_file, intensity_2d, delimiter=",", fmt="%.2f")
            
        else:
            csv_filename = f"results/{file_prefix}.csv"
            np.savetxt(csv_filename, self.raw_scan_data, delimiter=",", fmt="%.2f")
            self.controller.log(f"INFO: 2D Intensity Data saved to {csv_filename}")
        
        if self.fig:
            png_filename = f"results/{file_prefix}.png"
            self.ax.set_title(f"Saved: {file_prefix}")
            self.canvas.draw()
            self.fig.savefig(png_filename, dpi=150, bbox_inches='tight')
            self.controller.log(f"INFO: Image saved to {png_filename}")
            
    def get_scan_params(self,validate=True):
        try:
            p={k:float(e.get()) for k,e in self.scan_entries.items()}
            if validate and(p['steps_x']%1!=0 or p['steps_y']%1!=0 or p['steps_x']<1 or p['steps_y']<1):messagebox.showerror("Parameter Error","Points must be integers >= 1.");return None
            p['end_x']=p['start_x']+(p['steps_x']-1)*p['step_x'];p['end_y']=p['start_y']+(p['steps_y']-1)*p['step_y']
            return p
        except(ValueError,tk.TclError):
            if validate:messagebox.showerror("Parameter Error","All fields must contain valid numbers.");
            return None
            
    def connect(self):
        if self.controller.connect(self.conn_entries['port'].get(),int(self.conn_entries['baudrate'].get())):
            self.connect_button.config(state=tk.DISABLED);self.disconnect_button.config(state=tk.NORMAL);self.start_scan_button.config(state=tk.NORMAL)
            self._start_position_updater()
            
    def disconnect(self): 
        self._stop_position_updater(); self.controller.disconnect()
        self.connect_button.config(state=tk.NORMAL);self.disconnect_button.config(state=tk.DISABLED);self.start_scan_button.config(state=tk.DISABLED);self.stop_scan_button.config(state=tk.DISABLED)

    def stop_scan(self):
        if self.scan_thread and self.scan_thread.is_alive():self.controller.stop_scan()

    def on_closing(self): 
        self._stop_position_updater()
        if self.tt_device_instance: self.tt_device_instance.close()
        self.controller.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StageControlApp(root)
    root.mainloop()