# timetagger_imager.py
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import time
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.cm as cm

try:
    import TimeTagger
    TIMETAGGER_AVAILABLE = True
except ImportError:
    TIMETAGGER_AVAILABLE = False

SYNC_CHANNEL = 5
SYNC_TRIGGER_LEVEL = 1.7
APD_TRIGGER_LEVEL = 0.35
CHANNELS_TO_PING = [1, 2, 3, 4]

class TimeTaggerImagerWindow:
    def __init__(self, parent_root):
        self.window = tk.Toplevel(parent_root)
        self.window.title("Time Tagger Image Reconstructor")
        self.window.geometry("450x450")
        
        self.tagger = None
        self.ping_countrate = None
        self.is_listening = False
        
        self.cbm = None
        self.sync_counter = None
        self.delayed_sync = None
        
        self.image_data = None
        self.total_pixels = 0
        self.points_x = 0
        self.points_y = 0
        self.last_pixel_count = 0
        self.last_pixel_time = 0
        
        self.map_window = None
        self.fig = None
        self.ax = None
        self.im = None
        self.canvas = None

        self._setup_ui()
        self._connect_timetagger()
        
        self.window.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _setup_ui(self):
        main_frame = ttk.Frame(self.window, padding=15)
        main_frame.pack(fill=tk.BOTH, expand=True)

        param_lf = ttk.LabelFrame(main_frame, text="Scan Parameters", padding=10)
        param_lf.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(param_lf, text="Points X:").grid(row=0, column=0, sticky='w', pady=2)
        self.entry_x = ttk.Entry(param_lf, width=10)
        self.entry_x.insert(0, "100")
        self.entry_x.grid(row=0, column=1, pady=2, padx=5)
        
        ttk.Label(param_lf, text="Points Y:").grid(row=0, column=2, sticky='w', pady=2, padx=(10,0))
        self.entry_y = ttk.Entry(param_lf, width=10)
        self.entry_y.insert(0, "100")
        self.entry_y.grid(row=0, column=3, pady=2, padx=5)
        
        ttk.Label(param_lf, text="Dwell Time (ms):").grid(row=1, column=0, sticky='w', pady=2)
        self.entry_dwell = ttk.Entry(param_lf, width=10)
        self.entry_dwell.insert(0, "1.0")
        self.entry_dwell.grid(row=1, column=1, pady=2, padx=5)

        ping_lf = ttk.LabelFrame(main_frame, text="Active APD Channel (Live Ping)", padding=10)
        ping_lf.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.apd_var = tk.IntVar(value=1)
        self.lbl_rates = {}
        
        for i, ch in enumerate(CHANNELS_TO_PING):
            rb = ttk.Radiobutton(ping_lf, text=f"Channel {ch}", variable=self.apd_var, value=ch)
            rb.grid(row=i, column=0, sticky='w', pady=2)
            lbl = ttk.Label(ping_lf, text="0.00 Hz", width=15, anchor='e')
            lbl.grid(row=i, column=1, sticky='e', pady=2, padx=10)
            self.lbl_rates[ch] = lbl

        ttk.Separator(ping_lf, orient='horizontal').grid(row=len(CHANNELS_TO_PING), column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Label(ping_lf, text=f"SYNC (Ch {SYNC_CHANNEL}):", font='-weight bold').grid(row=len(CHANNELS_TO_PING)+1, column=0, sticky='w')
        self.lbl_sync_rate = ttk.Label(ping_lf, text="0.00 Hz", width=15, anchor='e', font='-weight bold')
        self.lbl_sync_rate.grid(row=len(CHANNELS_TO_PING)+1, column=1, sticky='e', padx=10)

        ctrl_frame = ttk.Frame(main_frame)
        ctrl_frame.pack(fill=tk.X)
        self.btn_start = ttk.Button(ctrl_frame, text="Start Listening", command=self._start_listening)
        self.btn_start.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.btn_stop = ttk.Button(ctrl_frame, text="Stop & Save", command=self._stop_listening, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.lbl_status = ttk.Label(main_frame, text="Status: Ready", foreground="gray")
        self.lbl_status.pack(pady=5, anchor='w')
        self.lbl_progress = ttk.Label(main_frame, text="Pixels: 0 / 0 (0%)")
        self.lbl_progress.pack(anchor='w')

    def _connect_timetagger(self):
        if not TIMETAGGER_AVAILABLE:
            self.lbl_status.config(text="Status: TimeTagger Lib Not Found", foreground="red")
            return
        try:
            self.lbl_status.config(text="Status: Connecting...", foreground="blue")
            self.window.update()
            self.tagger = TimeTagger.createTimeTagger()
            for ch in CHANNELS_TO_PING:
                self.tagger.setTriggerLevel(ch, APD_TRIGGER_LEVEL)
            self.tagger.setTriggerLevel(SYNC_CHANNEL, SYNC_TRIGGER_LEVEL)
            channels_all = CHANNELS_TO_PING + [SYNC_CHANNEL]
            self.ping_countrate = TimeTagger.Countrate(self.tagger, channels=channels_all)
            self.lbl_status.config(text=f"Status: Connected ({self.tagger.getModel()})", foreground="green")
            self._update_ping()
        except Exception as e:
            self.lbl_status.config(text="Status: Connection Failed", foreground="red")
            messagebox.showerror("Time Tagger Error", f"Failed to connect:\n{e}", parent=self.window)

    def _update_ping(self):
        if self.tagger and self.ping_countrate and not self.is_listening:
            try:
                rates = self.ping_countrate.getData()
                for i, ch in enumerate(CHANNELS_TO_PING):
                    self.lbl_rates[ch].config(text=f"{rates[i]:.2f} Hz")
                sync_rate = rates[-1]
                self.lbl_sync_rate.config(text=f"{sync_rate:.2f} Hz")
                if sync_rate > 0: self.lbl_sync_rate.config(foreground="orange")
                else: self.lbl_sync_rate.config(foreground="black")
            except Exception: pass
        # Пингуем через главный event loop, если окно не закрыто
        if self.window.winfo_exists():
            self.window.after(300, self._update_ping)

    def _start_listening(self):
        try:
            self.points_x = int(self.entry_x.get())
            self.points_y = int(self.entry_y.get())
            dwell_ms = float(self.entry_dwell.get())
            self.total_pixels = self.points_x * self.points_y
            apd_ch = self.apd_var.get()
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers.", parent=self.window)
            return

        if not self.tagger:
            messagebox.showerror("Error", "Time Tagger is not connected.", parent=self.window)
            return

        self.is_listening = True
        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self.lbl_status.config(text="Status: LISTENING (Waiting for TTL...)", foreground="orange")
        
        dwell_ps = int(dwell_ms * 1e9)
        self.delayed_sync = TimeTagger.DelayedChannel(self.tagger, SYNC_CHANNEL, dwell_ps)
        self.cbm = TimeTagger.CountBetweenMarkers(
            self.tagger, click_channel=apd_ch, begin_channel=SYNC_CHANNEL, 
            end_channel=self.delayed_sync.getChannel(), n_values=self.total_pixels
        )
        self.sync_counter = TimeTagger.Countrate(self.tagger, channels=[SYNC_CHANNEL])
        self.sync_counter.clear()
        
        self.image_data = np.full((self.points_y, self.points_x), np.nan)
        self.last_pixel_count = 0
        self.last_pixel_time = time.time()
        
        self._setup_plot_window()
        self._monitor_measurement()

    def _setup_plot_window(self):
        if self.map_window is not None and tk.Toplevel.winfo_exists(self.map_window):
            self.map_window.destroy()
            
        self.map_window = tk.Toplevel(self.window)
        self.map_window.title("Live APD Map")
        self.map_window.geometry("600x500")
        
        self.fig, self.ax = plt.subplots(figsize=(5, 4), dpi=100)
        self.fig.subplots_adjust(left=0.1, right=0.95, top=0.9, bottom=0.1)
        cmap = cm.get_cmap('viridis').copy()
        cmap.set_bad(color='#222222')
        self.im = self.ax.imshow(self.image_data, cmap=cmap, origin='upper', aspect='auto', interpolation='none')
        self.fig.colorbar(self.im, ax=self.ax, label="Photon Counts")
        self.ax.set_title("Waiting for scan to start...")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.map_window)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

    def _monitor_measurement(self):
        if not self.is_listening or not self.window.winfo_exists(): return

        current_pixels = int(self.sync_counter.getCountsTotal()[0])
        current_pixels = min(current_pixels, self.total_pixels)
        
        if current_pixels > self.last_pixel_count:
            self.lbl_status.config(text="Status: SCANNING", foreground="green")
            self.last_pixel_time = time.time()
            counts_array = self.cbm.getData()[:current_pixels]
            
            flat_image = np.full(self.total_pixels, np.nan)
            flat_image[:current_pixels] = counts_array
            self.image_data = flat_image.reshape((self.points_y, self.points_x))
            
            self.im.set_data(self.image_data)
            if len(counts_array) > 0:
                vmax = np.percentile(counts_array, 99.9)
                vmin = np.min(counts_array)
                if vmax <= vmin: vmax = vmin + 1
                self.im.set_clim(vmin, vmax)
            
            self.ax.set_title(f"Scanning... {current_pixels}/{self.total_pixels}")
            self.canvas.draw_idle()
            
            pct = (current_pixels / self.total_pixels) * 100
            self.lbl_progress.config(text=f"Pixels: {current_pixels} / {self.total_pixels} ({pct:.1f}%)")
            self.last_pixel_count = current_pixels

        if current_pixels >= self.total_pixels:
            self.lbl_status.config(text="Status: Scan Completed", foreground="blue")
            self._stop_listening()
            return
            
        if current_pixels > 0 and current_pixels < self.total_pixels:
            dwell_ms = float(self.entry_dwell.get())
            timeout_threshold = (dwell_ms * 3 / 1000.0) + 2.0 
            if (time.time() - self.last_pixel_time) > timeout_threshold:
                self.lbl_status.config(text="Status: Scan Aborted (Timeout)", foreground="red")
                self._stop_listening()
                return

        self.window.after(100, self._monitor_measurement)

    def _stop_listening(self):
        if not self.is_listening: return
        self.is_listening = False
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        if self.last_pixel_count > 0: self._save_results()
        self.cbm = None
        self.sync_counter = None
        self.delayed_sync = None
        self._update_ping()

 #параметры дрбавить
    def _save_results(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("results", exist_ok=True)
        csv_filename = f"results/scan_{timestamp}.csv"
        png_filename = f"results/scan_{timestamp}.png"
        np.savetxt(csv_filename, self.image_data, delimiter=",", fmt="%.2f")
        if self.fig:
            self.ax.set_title("Scan Result")
            self.canvas.draw()
            self.fig.savefig(png_filename, dpi=150, bbox_inches='tight')

    def _on_closing(self):
        self.is_listening = False
        if self.tagger:
            print("Отключение Time Tagger...")
            TimeTagger.freeTimeTagger(self.tagger)
        self.window.destroy()

if __name__ == "__main__":
    test_root = tk.Tk()
    test_root.withdraw()
    app = TimeTaggerImagerWindow(test_root)
    test_root.mainloop()