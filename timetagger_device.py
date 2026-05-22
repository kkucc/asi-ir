# timetagger_device.py
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import time

try:
    import TimeTagger
    TIMETAGGER_AVAILABLE = True
except ImportError:
    TIMETAGGER_AVAILABLE = False

class TimeTaggerDevice:
    def __init__(self):
        self.tagger = None
        self.is_connected = False
        
        self.sync_channel = 5
        self.apd_channel = 1
        self.laser_channel = 3  
        
        self.sync_trigger = 1.70
        self.apd_trigger = -0.25
        self.laser_trigger = 1.00 
        
        self.gating_enabled = False
        self.gate_start_ns = 2.0
        self.gate_stop_ns = 10.0
        
        self.flim_binwidth_ps = 1000
        self.flim_n_bins = 1000
        self.is_flim_mode = False
        
        self.ping_measurement = None
        self.pixel_measurement = None  
        
        self.delay_open = None
        self.delay_close = None
        self.gated_apd = None
        
        self.connect()

    def connect(self):
        if not TIMETAGGER_AVAILABLE or self.is_connected: return
        try:
            self.tagger = TimeTagger.createTimeTagger()
            self.apply_triggers()
            self.ping_measurement = TimeTagger.Counter(
                self.tagger, channels=[1, 2, 3, 4, 5], binwidth=int(2e11), n_values=1
            )
            self.is_connected = True
            print(f"[TimeTagger] Успешное подключение. Модель: {self.tagger.getModel()}")
        except Exception as e:
            print(f"[TimeTagger] Ошибка подключения: {e}")
            self.is_connected = False

    def apply_triggers(self):
        if self.tagger:
            self.tagger.setTriggerLevel(self.sync_channel, self.sync_trigger)
            self.tagger.setTriggerLevel(self.laser_channel, self.laser_trigger)
            for ch in [1, 2, 4]:
                self.tagger.setTriggerLevel(ch, self.apd_trigger)

    def setup_scan(self, total_pixels: int, dwell_s: float, mode: str = "Intensity"):
        if not self.is_connected: return
        
        self.is_flim_mode = (mode == "FLIM")
        dwell_ps = int(dwell_s * 1e12)
        active_apd = self.apd_channel
        
        if self.gating_enabled:
            print(f"[TimeTagger] Time Gating ВКЛЮЧЕН: {self.gate_start_ns}ns - {self.gate_stop_ns}ns")
            start_ps = int(self.gate_start_ns * 1000)
            stop_ps = int(self.gate_stop_ns * 1000)
            self.delay_open = TimeTagger.DelayedChannel(self.tagger, self.laser_channel, start_ps)
            self.delay_close = TimeTagger.DelayedChannel(self.tagger, self.laser_channel, stop_ps)
            self.gated_apd = TimeTagger.GatedChannel(
                self.tagger, input_channel=self.apd_channel,
                gate_start_channel=self.delay_open.getChannel(),
                gate_stop_channel=self.delay_close.getChannel()
            )
            active_apd = self.gated_apd.getChannel()
            
        if self.is_flim_mode:
            print(f"[TimeTagger] Режим FLIM: Bin {self.flim_binwidth_ps}ps -> {self.flim_n_bins} Bins")
            self.pixel_measurement = TimeTagger.Histogram(
                self.tagger, click_channel=active_apd, start_channel=self.laser_channel,
                binwidth=self.flim_binwidth_ps, n_bins=self.flim_n_bins
            )
        else:
            print("[TimeTagger] Режим Intensity")
            self.pixel_measurement = TimeTagger.Counter(
                self.tagger, channels=[active_apd], binwidth=dwell_ps, n_values=1
            )

    def acquire(self, dwell_s: float, x: float, y: float):
        if not self.is_connected or not self.pixel_measurement:
            time.sleep(dwell_s)
            return np.full(self.flim_n_bins, np.nan) if self.is_flim_mode else np.nan

        dwell_ps = int(dwell_s * 1e12)
        self.pixel_measurement.startFor(dwell_ps, clear=True)
        self.pixel_measurement.waitUntilFinished()
        data = self.pixel_measurement.getData()
        
        if self.is_flim_mode:
            return np.array(data)
        else:
            return float(data[0])

    def teardown_scan(self):
        if self.pixel_measurement: self.pixel_measurement.stop()
        if self.gated_apd: self.gated_apd.stop()
        if self.delay_close: self.delay_close.stop()
        if self.delay_open: self.delay_open.stop()
        self.pixel_measurement = None
        self.gated_apd = None; self.delay_close = None; self.delay_open = None

    def close(self):
        if self.tagger:
            if self.ping_measurement: self.ping_measurement.stop()
            TimeTagger.freeTimeTagger(self.tagger)
            self.tagger = None; self.is_connected = False
            
    def __str__(self): return "TimeTagger"


class TTSettingsPopup:
    def __init__(self, parent, tt_device):
        self.tt = tt_device
        self.window = tk.Toplevel(parent)
        self.window.title("Time Tagger Settings")
        self.window.geometry("400x480")
        self.window.resizable(False, False)
        self.window.transient(parent)
        self._setup_ui()
        self._update_ping()

    def _setup_ui(self):
        main_frame = ttk.Frame(self.window, padding=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        trig_lf = ttk.LabelFrame(main_frame, text="Trigger Levels (V)", padding=10)
        trig_lf.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(trig_lf, text="APD:").grid(row=0, column=0, sticky='w')
        self.ent_apd_trig = ttk.Entry(trig_lf, width=5); self.ent_apd_trig.insert(0, f"{self.tt.apd_trigger:.2f}")
        self.ent_apd_trig.grid(row=0, column=1, padx=2)
        ttk.Label(trig_lf, text="SYNC:").grid(row=0, column=2, sticky='w', padx=(5,0))
        self.ent_sync_trig = ttk.Entry(trig_lf, width=5); self.ent_sync_trig.insert(0, f"{self.tt.sync_trigger:.2f}")
        self.ent_sync_trig.grid(row=0, column=3, padx=2)
        ttk.Label(trig_lf, text="Laser:").grid(row=0, column=4, sticky='w', padx=(5,0))
        self.ent_laser_trig = ttk.Entry(trig_lf, width=5); self.ent_laser_trig.insert(0, f"{self.tt.laser_trigger:.2f}")
        self.ent_laser_trig.grid(row=0, column=5, padx=2)

        gate_lf = ttk.LabelFrame(main_frame, text="Hardware Time Gating (TCSPC)", padding=10)
        gate_lf.pack(fill=tk.X, pady=(0, 5))
        self.gate_var = tk.BooleanVar(value=self.tt.gating_enabled)
        ttk.Checkbutton(gate_lf, text="Enable Time Gating", variable=self.gate_var).grid(row=0, column=0, columnspan=4, sticky='w', pady=(0,5))
        ttk.Label(gate_lf, text="Start (ns):").grid(row=1, column=0, sticky='w')
        self.ent_gstart = ttk.Entry(gate_lf, width=6); self.ent_gstart.insert(0, f"{self.tt.gate_start_ns:.1f}")
        self.ent_gstart.grid(row=1, column=1, padx=5)
        ttk.Label(gate_lf, text="Stop (ns):").grid(row=1, column=2, sticky='w', padx=(10,0))
        self.ent_gstop = ttk.Entry(gate_lf, width=6); self.ent_gstop.insert(0, f"{self.tt.gate_stop_ns:.1f}")
        self.ent_gstop.grid(row=1, column=3, padx=5)

        flim_lf = ttk.LabelFrame(main_frame, text="FLIM Histogram Parameters", padding=10)
        flim_lf.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(flim_lf, text="Bin Width (ps):").grid(row=0, column=0, sticky='w')
        self.ent_binw = ttk.Entry(flim_lf, width=6); self.ent_binw.insert(0, str(self.tt.flim_binwidth_ps))
        self.ent_binw.grid(row=0, column=1, padx=5)
        ttk.Label(flim_lf, text="N Bins:").grid(row=0, column=2, sticky='w', padx=(10,0))
        self.ent_nbins = ttk.Entry(flim_lf, width=6); self.ent_nbins.insert(0, str(self.tt.flim_n_bins))
        self.ent_nbins.grid(row=0, column=3, padx=5)

        ttk.Button(main_frame, text="Apply All Settings", command=self._apply_all).pack(fill=tk.X, pady=(0, 10))

        ping_lf = ttk.LabelFrame(main_frame, text="Active APD Channel (Live Ping)", padding=10)
        ping_lf.pack(fill=tk.BOTH, expand=True)
        self.apd_var = tk.IntVar(value=self.tt.apd_channel)
        self.lbl_rates = {}
        for i, ch in enumerate([1, 2, 4]):
            rb = ttk.Radiobutton(ping_lf, text=f"Channel {ch} (APD)", variable=self.apd_var, value=ch, command=self._set_channel)
            rb.grid(row=i, column=0, sticky='w', pady=2)
            lbl = ttk.Label(ping_lf, text="0.00 Hz", width=12, anchor='e')
            lbl.grid(row=i, column=1, sticky='e', pady=2, padx=10)
            self.lbl_rates[ch] = lbl
        ttk.Separator(ping_lf, orient='horizontal').grid(row=3, column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Label(ping_lf, text="Ch 3 (Laser):", font='-weight bold').grid(row=4, column=0, sticky='w')
        self.lbl_rates[3] = ttk.Label(ping_lf, text="0.00 Hz", width=12, anchor='e', font='-weight bold')
        self.lbl_rates[3].grid(row=4, column=1, sticky='e', padx=10)
        ttk.Label(ping_lf, text="Ch 5 (Sync):", font='-weight bold').grid(row=5, column=0, sticky='w')
        self.lbl_rates[5] = ttk.Label(ping_lf, text="0.00 Hz", width=12, anchor='e', font='-weight bold')
        self.lbl_rates[5].grid(row=5, column=1, sticky='e', padx=10)

    def _apply_all(self):
        try:
            self.tt.apd_trigger = float(self.ent_apd_trig.get())
            self.tt.sync_trigger = float(self.ent_sync_trig.get())
            self.tt.laser_trigger = float(self.ent_laser_trig.get())
            self.tt.gating_enabled = self.gate_var.get()
            self.tt.gate_start_ns = float(self.ent_gstart.get())
            self.tt.gate_stop_ns = float(self.ent_gstop.get())
            
            self.tt.flim_binwidth_ps = int(self.ent_binw.get())
            self.tt.flim_n_bins = int(self.ent_nbins.get())
            
            self.tt.apply_triggers()
            messagebox.showinfo("Success", "Settings applied successfully!", parent=self.window)
        except ValueError:
            messagebox.showerror("Error", "Invalid numerical values.", parent=self.window)

    def _set_channel(self): self.tt.apd_channel = self.apd_var.get()

    def _update_ping(self):
        if not self.window.winfo_exists(): return
        if self.tt.is_connected and self.tt.ping_measurement:
            try:
                counts_matrix = self.tt.ping_measurement.getData()
                for i, ch in enumerate([1, 2, 3, 4, 5]):
                    rate_hz = counts_matrix[i][0] * 5.0
                    self.lbl_rates[ch].config(text=f"{rate_hz:.2f} Hz")
                    if ch in [3, 5]:
                        self.lbl_rates[ch].config(foreground="orange" if rate_hz > 0 else "black")
            except Exception: pass
        self.window.after(200, self._update_ping)