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
        self.sync_trigger = 1.70
        self.apd_trigger = -0.25
        
        self.ping_measurement = None
        self.pixel_counter = None  
        
        self.connect()

    def connect(self):
        if not TIMETAGGER_AVAILABLE or self.is_connected: return
        try:
            self.tagger = TimeTagger.createTimeTagger()
            self.apply_triggers()
            
            # Пинг: Окно 200 мс для мгновенной реакции на свет
            self.ping_measurement = TimeTagger.Counter(
                self.tagger, 
                channels=[1, 2, 3, 4, 5], 
                binwidth=int(2e11), 
                n_values=1
            )
            
            self.is_connected = True
            print(f"[TimeTagger] Успешное подключение. Модель: {self.tagger.getModel()}")
        except Exception as e:
            print(f"[TimeTagger] Ошибка подключения: {e}")
            self.is_connected = False

    def apply_triggers(self):
        if self.tagger:
            self.tagger.setTriggerLevel(self.sync_channel, self.sync_trigger)
            for ch in[1, 2, 3, 4]:
                self.tagger.setTriggerLevel(ch, self.apd_trigger)

    def setup_scan(self, total_pixels: int, dwell_s: float):
        """Подготовка: создаем один счетчик для точного измерения пикселя"""
        if not self.is_connected: return
        
        dwell_ps = int(dwell_s * 1e12)
        
        self.pixel_counter = TimeTagger.Counter(
            self.tagger,
            channels=[self.apd_channel],
            binwidth=dwell_ps,
            n_values=1
        )

    def acquire(self, dwell_s: float, x: float, y: float) -> float:
        """Считывает фотоны, пока столик стоит на месте."""
        if not self.is_connected or not self.pixel_counter:
            time.sleep(dwell_s)
            return np.nan

        dwell_ps = int(dwell_s * 1e12)
        
        self.pixel_counter.startFor(dwell_ps, clear=True)
        
        self.pixel_counter.waitUntilFinished()
        
        counts = self.pixel_counter.getData()[0]
        
        print(f"Пиксель [{x:.1f}, {y:.1f}] -> Фотоны: {counts}")
        
        return float(counts)

    def teardown_scan(self):
        """Очистка после скана."""
        if self.pixel_counter:
            self.pixel_counter.stop()
        self.pixel_counter = None

    def close(self):
        if self.tagger:
            if self.ping_measurement:
                self.ping_measurement.stop()
            TimeTagger.freeTimeTagger(self.tagger)
            self.tagger = None
            self.is_connected = False
            
    def __str__(self):
        return "TimeTagger"


class TTSettingsPopup:
    def __init__(self, parent, tt_device):
        self.tt = tt_device
        self.window = tk.Toplevel(parent)
        self.window.title("Time Tagger Settings")
        self.window.geometry("380x300")
        self.window.resizable(False, False)
        self.window.transient(parent)
        
        self._setup_ui()
        self._update_ping()

    def _setup_ui(self):
        main_frame = ttk.Frame(self.window, padding=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        trig_lf = ttk.LabelFrame(main_frame, text="Trigger Levels (V)", padding=10)
        trig_lf.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(trig_lf, text="APD (1-4):").grid(row=0, column=0, sticky='w')
        self.ent_apd_trig = ttk.Entry(trig_lf, width=6)
        self.ent_apd_trig.insert(0, f"{self.tt.apd_trigger:.2f}")
        self.ent_apd_trig.grid(row=0, column=1, padx=5)
        
        ttk.Label(trig_lf, text="SYNC (5):").grid(row=0, column=2, sticky='w', padx=(10,0))
        self.ent_sync_trig = ttk.Entry(trig_lf, width=6)
        self.ent_sync_trig.insert(0, f"{self.tt.sync_trigger:.2f}")
        self.ent_sync_trig.grid(row=0, column=3, padx=5)
        
        ttk.Button(trig_lf, text="Apply", command=self._apply_triggers, width=6).grid(row=0, column=4, padx=(10,0))

        ping_lf = ttk.LabelFrame(main_frame, text="Active APD Channel (Live Ping)", padding=10)
        ping_lf.pack(fill=tk.BOTH, expand=True)
        
        self.apd_var = tk.IntVar(value=self.tt.apd_channel)
        self.lbl_rates = {}
        
        for i, ch in enumerate([1, 2, 3, 4]):
            rb = ttk.Radiobutton(ping_lf, text=f"Channel {ch}", variable=self.apd_var, value=ch, command=self._set_channel)
            rb.grid(row=i, column=0, sticky='w', pady=2)
            lbl = ttk.Label(ping_lf, text="0.00 Hz", width=12, anchor='e')
            lbl.grid(row=i, column=1, sticky='e', pady=2, padx=10)
            self.lbl_rates[ch] = lbl

        ttk.Separator(ping_lf, orient='horizontal').grid(row=4, column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Label(ping_lf, text="SYNC (Ch 5):", font='-weight bold').grid(row=5, column=0, sticky='w')
        self.lbl_sync_rate = ttk.Label(ping_lf, text="0.00 Hz", width=12, anchor='e', font='-weight bold')
        self.lbl_sync_rate.grid(row=5, column=1, sticky='e', padx=10)

    def _apply_triggers(self):
        try:
            self.tt.apd_trigger = float(self.ent_apd_trig.get())
            self.tt.sync_trigger = float(self.ent_sync_trig.get())
            self.tt.apply_triggers()
        except ValueError:
            messagebox.showerror("Error", "Invalid trigger values.", parent=self.window)

    def _set_channel(self):
        self.tt.apd_channel = self.apd_var.get()

    def _update_ping(self):
        if not self.window.winfo_exists(): return
        
        if self.tt.is_connected and self.tt.ping_measurement:
            try:
                counts_matrix = self.tt.ping_measurement.getData()
                for i, ch in enumerate([1, 2, 3, 4]):
                    rate_hz = counts_matrix[i][0] * 5.0
                    self.lbl_rates[ch].config(text=f"{rate_hz:.2f} Hz")
                
                sync_rate_hz = counts_matrix[4][0] * 5.0
                self.lbl_sync_rate.config(text=f"{sync_rate_hz:.2f} Hz")
                self.lbl_sync_rate.config(foreground="orange" if sync_rate_hz > 0 else "black")
            except Exception:
                pass
                
        self.window.after(200, self._update_ping)