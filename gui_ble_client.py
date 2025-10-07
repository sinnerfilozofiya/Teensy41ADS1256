# ble_loadcell_gui.py
# GUI for ESP32 Load Cell BLE Server with:
# - Scan & device selection
# - Connect/Disconnect
# - Commands + custom command
# - Raw values (8 channels) live monitor
# - Live graph for a selected channel (L1..L4, R5..R8)
#
# Spec: Data notifications contain 1..10 samples/packet, each sample = 8 x int16 LE
#       Order: local[4], remote[4] => L1..L4, R5..R8

import asyncio
import struct
import threading
import queue
import sys
import time
from collections import deque
from typing import Optional, List, Dict

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:
    print("Tkinter not available. Please install/enable Tk support.")
    sys.exit(1)

from bleak import BleakClient, BleakScanner, BleakError

# Matplotlib embed
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ==== UUIDs (from your doc) ====
SERVICE_UUID       = "12345678-1234-1234-1234-123456789abc"
DATA_CHAR_UUID     = "87654321-4321-4321-4321-cba987654321"  # Notify + Read
COMMAND_CHAR_UUID  = "11111111-2222-3333-4444-555555555555"  # Write

DEFAULT_DEVICE_NAME = "LoadCell_BLE_Server"

# ==== Command sets (from your doc) ====
COMMAND_GROUPS = {
    "Local": ["START", "STOP", "RESTART", "RESET"],
    "Remote": ["REMOTE_START", "REMOTE_STOP", "REMOTE_RESTART", "REMOTE_RESET"],
    "Dual": ["ALL_START", "ALL_STOP", "ALL_RESTART", "ALL_RESET"],
    "Zero (Local)": ["ZERO", "ZERO_STATUS", "ZERO_RESET"],
    "Zero (Remote)": ["REMOTE_ZERO", "REMOTE_ZERO_STATUS", "REMOTE_ZERO_RESET"],
    "Zero (All)": ["ALL_ZERO", "ALL_ZERO_STATUS", "ALL_ZERO_RESET"],
    "System": ["STATUS", "LOCAL_ON", "LOCAL_OFF", "REMOTE_ON", "REMOTE_OFF"],
}

CHANNEL_NAMES = ["L1", "L2", "L3", "L4", "R5", "R6", "R7", "R8"]

# ===== Packet parser =====
def parse_packet(data: bytes):
    """Return {"sample_count": int, "samples": [[8 int16], ...]}"""
    if not data or len(data) < 1:
        return {"sample_count": 0, "samples": []}
    sample_count = data[0]
    expected_len = 1 + sample_count * 16
    if len(data) < expected_len or sample_count == 0:
        return {"sample_count": 0, "samples": []}

    samples: List[List[int]] = []
    for i in range(sample_count):
        off = 1 + i * 16
        local4  = struct.unpack_from("<4h", data, off)
        remote4 = struct.unpack_from("<4h", data, off + 8)
        samples.append(list(local4) + list(remote4))
    return {"sample_count": sample_count, "samples": samples}

# ===== BLE worker thread (async) =====
class BLEWorker:
    def __init__(self, ui_q: queue.Queue):
        self.client: Optional[BleakClient] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self.ui_q = ui_q
        self.selected_address: Optional[str] = None
        self.packet_counter = 0

    def send_ui(self, kind: str, payload):
        self.ui_q.put({"kind": kind, "payload": payload})

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()

    def stop(self):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._shutdown(), self.loop)

    async def _shutdown(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                self.send_ui("status", "Disconnected.")
        except Exception as e:
            self.send_ui("log", f"Error during disconnect: {e}")
        finally:
            await asyncio.sleep(0.05)
            if self.loop.is_running():
                self.loop.stop()

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.send_ui("status", "BLE thread started.")
        try:
            self.loop.run_forever()
        except Exception as e:
            self.send_ui("log", f"Loop error: {e}")
        finally:
            self.send_ui("status", "BLE thread stopped.")

    # ----- public: called from UI thread -----
    def scan(self, use_service_filter: bool, name_hint: str):
        asyncio.run_coroutine_threadsafe(
            self._scan_task(use_service_filter, name_hint), self.loop
        )

    def connect(self, address: Optional[str]):
        self.selected_address = address
        asyncio.run_coroutine_threadsafe(self._connect_task(), self.loop)

    def disconnect(self):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._disconnect_task(), self.loop)

    def send_command(self, cmd: str):
        asyncio.run_coroutine_threadsafe(self._send_command_task(cmd), self.loop)

    # ----- async tasks -----
    async def _scan_task(self, use_service_filter: bool, name_hint: str):
        self.send_ui("status", "Scanning for devices...")
        try:
            devices = await BleakScanner.discover(timeout=5.0)
        except Exception as e:
            self.send_ui("status", f"Scan error: {e}")
            self.send_ui("devices", [])
            return

        results = []
        name_hint_l = (name_hint or "").strip().lower()
        for d in devices:
            nm = d.name or ""
            show = True
            if name_hint_l and name_hint_l not in nm.lower():
                show = False

            if use_service_filter:
                adv_uuids = set()
                try:
                    if hasattr(d, "metadata") and d.metadata and "uuids" in d.metadata:
                        adv_uuids = set(d.metadata.get("uuids") or [])
                except Exception:
                    adv_uuids = set()
                if SERVICE_UUID.lower() not in [u.lower() for u in adv_uuids]:
                    if not name_hint_l:
                        show = False

            if show:
                results.append({
                    "name": nm if nm else "(unknown)",
                    "address": d.address,
                    "rssi": getattr(d, "rssi", None),
                })

        results.sort(key=lambda x: (x["rssi"] is None, -(x["rssi"] or -999), x["name"]))
        self.send_ui("devices", results)
        self.send_ui("status", f"Scan complete. Found {len(results)} device(s).")

    async def _connect_task(self):
        if not self.selected_address:
            self.send_ui("status", "No device selected.")
            return
        if self.client and self.client.is_connected:
            self.send_ui("status", "Already connected.")
            return

        self.client = BleakClient(self.selected_address)
        try:
            await self.client.connect()
            self.send_ui("status", f"Connected: {self.selected_address}")
            await self.client.start_notify(DATA_CHAR_UUID, self._notification_handler)
            self.packet_counter = 0
            self.send_ui("status", "Notifications enabled.")
        except BleakError as e:
            self.send_ui("status", f"Connect error: {e}")
            self.client = None
        except Exception as e:
            self.send_ui("status", f"Connect error: {e}")
            self.client = None

    async def _disconnect_task(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.stop_notify(DATA_CHAR_UUID)
                await self.client.disconnect()
                self.send_ui("status", "Disconnected.")
        except Exception as e:
            self.send_ui("log", f"Disconnect error: {e}")
        finally:
            self.client = None

    async def _send_command_task(self, cmd: str):
        cmd = (cmd or "").strip()
        if not cmd:
            self.send_ui("log", "Empty command.")
            return
        if not self.client or not self.client.is_connected:
            self.send_ui("log", "Not connected.")
            return
        try:
            await self.client.write_gatt_char(COMMAND_CHAR_UUID, cmd.encode("utf-8"))
            self.send_ui("log", f">> {cmd}")
        except Exception as e:
            self.send_ui("log", f"Command error: {e}")

    # ----- notification handler -----
    def _notification_handler(self, sender, data: bytearray):
        parsed = parse_packet(bytes(data))
        if not parsed["sample_count"]:
            return

        self.packet_counter += 1
        samples = parsed["samples"]  # list of [8 ints]
        # push raw samples to UI (for plotting + raw values)
        self.send_ui("samples", samples)

        # also send a terse textual summary (like before)
        mins = [min(s[ch] for s in samples) for ch in range(8)]
        maxs = [max(s[ch] for s in samples) for ch in range(8)]
        first = samples[0]; last = samples[-1]
        line1 = (f"[pkt {self.packet_counter:06d}] samples={parsed['sample_count']}  "
                 f"first L={first[0:4]} R={first[4:8]}  "
                 f"last L={last[0:4]} R={last[4:8]}")
        line2 = (f"           mins L={mins[0:4]} R={mins[4:8]}  "
                 f"maxs L={maxs[0:4]} R={maxs[4:8]}")
        self.send_ui("data", line1)
        self.send_ui("data", line2)

# ===== Main Tk app =====
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Load Cell BLE Monitor")
        self.geometry("1200x820")

        # Cross-thread queue
        self.q = queue.Queue()

        # BLE worker
        self.ble = BLEWorker(self.q)
        self.ble.start()

        # Device list cache
        self.device_map: Dict[str, str] = {}

        # Ring buffers for plots (per channel)
        self.sample_rate_hz = 1000  # overall per channel rate
        self.window_seconds = tk.IntVar(value=10)
        self.max_points = self.sample_rate_hz * self.window_seconds.get()
        self.buffers = [deque(maxlen=self.max_points) for _ in range(8)]
        self.sample_index = 0  # monotonically increasing index for x-axis
        self.paused = tk.BooleanVar(value=False)

        # Latest raw values
        self.latest_vals = [0]*8

        # Selected channel
        self.selected_channel = tk.StringVar(value=CHANNEL_NAMES[0])  # default L1

        # ------------- UI Layout -------------
        self._build_top_bar()
        self._build_scan_select()
        self._build_commands()
        self._build_monitor_and_plot()
        self._build_output()

        # Timers
        self.after(50, self._pump_queue)
        self.after(100, self._update_plot)

        self._log("Ready. Scan, select a device, Connect, then use commands.")

    # ---- UI sections ----
    def _build_top_bar(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=10)

        self.status_var = tk.StringVar(value="Idle.")
        ttk.Label(top, textvariable=self.status_var).pack(side="left")

        btns = ttk.Frame(top)
        btns.pack(side="right")
        ttk.Button(btns, text="Disconnect", command=self.ble.disconnect).pack(side="right", padx=4)
        ttk.Button(btns, text="Connect", command=self._connect_selected).pack(side="right", padx=4)

    def _build_scan_select(self):
        scan_frame = ttk.LabelFrame(self, text="Scan & Select Device")
        scan_frame.pack(fill="x", padx=10, pady=6)

        left = ttk.Frame(scan_frame)
        left.pack(side="left", fill="x", expand=True, padx=6, pady=6)

        self.filter_var = tk.StringVar(value=DEFAULT_DEVICE_NAME)
        ttk.Label(left, text="Name filter (optional):").pack(side="left")
        self.filter_entry = ttk.Entry(left, textvariable=self.filter_var, width=28)
        self.filter_entry.pack(side="left", padx=6)

        self.use_service_filter = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="Filter by Service UUID", variable=self.use_service_filter).pack(side="left", padx=6)

        ttk.Button(scan_frame, text="Scan", command=self._scan_devices).pack(side="right", padx=6)

        sel_row = ttk.Frame(self)
        sel_row.pack(fill="x", padx=10, pady=6)
        ttk.Label(sel_row, text="Select device:").pack(side="left")
        self.device_combo = ttk.Combobox(sel_row, state="readonly", width=90)
        self.device_combo.pack(side="left", fill="x", expand=True, padx=6)

    def _build_commands(self):
        cmd_frame = ttk.LabelFrame(self, text="Commands")
        cmd_frame.pack(fill="x", padx=10, pady=6)

        for grp, cmds in COMMAND_GROUPS.items():
            row = ttk.Frame(cmd_frame)
            row.pack(fill="x", padx=4, pady=2)
            ttk.Label(row, text=grp, width=14).pack(side="left")
            for c in cmds:
                ttk.Button(row, text=c, command=lambda cc=c: self.ble.send_command(cc)).pack(side="left", padx=2, pady=2)

        custom = ttk.Frame(self)
        custom.pack(fill="x", padx=10, pady=6)
        ttk.Label(custom, text="Custom Command:").pack(side="left")
        self.custom_entry = ttk.Entry(custom)
        self.custom_entry.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(custom, text="Send", command=self._send_custom).pack(side="left")

    def _build_monitor_and_plot(self):
        holder = ttk.Frame(self)
        holder.pack(fill="both", expand=True, padx=10, pady=6)

        # Left: Raw values table
        left = ttk.LabelFrame(holder, text="Raw Values (latest int16)")
        left.pack(side="left", fill="y", padx=6, pady=6)

        self.val_labels = []
        for i, name in enumerate(CHANNEL_NAMES):
            row = ttk.Frame(left)
            row.pack(fill="x", padx=6, pady=2)
            ttk.Label(row, text=f"{name}:", width=6).pack(side="left")
            lbl = ttk.Label(row, text="0", width=12)
            lbl.pack(side="left")
            self.val_labels.append(lbl)

        # Right: Plot
        right = ttk.LabelFrame(holder, text="Live Plot")
        right.pack(side="left", fill="both", expand=True, padx=6, pady=6)

        control = ttk.Frame(right)
        control.pack(fill="x", padx=6, pady=6)

        ttk.Label(control, text="Channel:").pack(side="left")
        self.channel_combo = ttk.Combobox(control, state="readonly", values=CHANNEL_NAMES, width=6,
                                          textvariable=self.selected_channel)
        self.channel_combo.pack(side="left", padx=6)

        ttk.Label(control, text="Window (s):").pack(side="left")
        window_box = ttk.Combobox(control, state="readonly", values=[5,10,30], width=5,
                                  textvariable=self.window_seconds)
        window_box.pack(side="left", padx=6)
        window_box.bind("<<ComboboxSelected>>", self._on_window_change)

        ttk.Checkbutton(control, text="Pause", variable=self.paused).pack(side="left", padx=12)

        # Matplotlib figure
        self.fig = Figure(figsize=(7, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Selected channel")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Raw int16")
        self.line, = self.ax.plot([], [])  # default color/style

        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_output(self):
        out_frame = ttk.LabelFrame(self, text="Log / Output")
        out_frame.pack(fill="both", expand=True, padx=10, pady=6)

        self.text = tk.Text(out_frame, wrap="none", height=10)
        self.text.pack(side="left", fill="both", expand=True)
        yscroll = ttk.Scrollbar(out_frame, command=self.text.yview)
        yscroll.pack(side="right", fill="y")
        self.text.configure(yscrollcommand=yscroll.set)

    # ---- actions ----
    def _scan_devices(self):
        self._log("Scanning for 5 seconds...")
        self.ble.scan(self.use_service_filter.get(), self.filter_entry.get())

    def _connect_selected(self):
        disp = (self.device_combo.get() or "").strip()
        if not disp or disp == "<no devices>":
            messagebox.showinfo("Select device", "Please scan and select a device first.")
            return
        addr = self.device_map.get(disp)
        if not addr:
            messagebox.showerror("Error", "Selected entry has no address.")
            return
        self._log(f"Connecting to {disp} ...")
        self.ble.connect(addr)

    def _send_custom(self):
        cmd = self.custom_entry.get().strip()
        if cmd:
            self.ble.send_command(cmd)

    def _on_window_change(self, _evt=None):
        # Resize ring buffers
        new_secs = int(self.window_seconds.get())
        self.max_points = self.sample_rate_hz * new_secs
        # rebuild buffers preserving recent tail
        for ch in range(8):
            old = list(self.buffers[ch])
            self.buffers[ch] = deque(old[-self.max_points:], maxlen=self.max_points)
        self._log(f"Plot window set to {new_secs}s ({self.max_points} samples).")

    # ---- queue + UI updates ----
    def _pump_queue(self):
        try:
            while True:
                msg = self.q.get_nowait()
                kind = msg.get("kind")
                payload = msg.get("payload")
                if kind == "status":
                    self.status_var.set(payload)
                    self._log(f"[status] {payload}")
                elif kind == "log":
                    self._log(str(payload))
                elif kind == "data":
                    self._print_line(str(payload))
                elif kind == "devices":
                    self._populate_devices(payload or [])
                elif kind == "samples":
                    self._ingest_samples(payload or [])
        except queue.Empty:
            pass
        self.after(50, self._pump_queue)

    def _populate_devices(self, items: List[Dict]):
        self.device_map.clear()
        display_list = []
        for it in items:
            name = it.get("name") or "(unknown)"
            addr = it.get("address") or "?"
            rssi = it.get("rssi")
            disp = f"{name} ({addr})" + (f"  [RSSI {rssi}]" if rssi is not None else "")
            self.device_map[disp] = addr
            display_list.append(disp)
        if not display_list:
            display_list = ["<no devices>"]
            self.device_map[display_list[0]] = ""
        self.device_combo["values"] = display_list
        if display_list:
            self.device_combo.set(display_list[0])

    def _ingest_samples(self, samples: List[List[int]]):
        # samples: list of 8-int rows
        for row in samples:
            self.sample_index += 1
            for ch in range(8):
                self.buffers[ch].append(row[ch])
                self.latest_vals[ch] = row[ch]
        # update raw values panel
        for ch in range(8):
            self.val_labels[ch].configure(text=str(self.latest_vals[ch]))

    def _update_plot(self):
        try:
            if not self.paused.get():
                ch_name = self.selected_channel.get()
                ch_idx = CHANNEL_NAMES.index(ch_name) if ch_name in CHANNEL_NAMES else 0
                y = list(self.buffers[ch_idx])
                if y:
                    x = list(range(len(y)))  # sample indices in window
                    self.line.set_data(x, y)
                    self.ax.relim()
                    self.ax.autoscale_view()
                self.canvas.draw_idle()
        except Exception as e:
            self._log(f"Plot update error: {e}")
        # refresh rate ~10 Hz is enough (lighter on CPU); tune as needed
        self.after(100, self._update_plot)

    # ---- helpers ----
    def _print_line(self, s: str):
        self.text.insert("end", s + "\n")
        self.text.see("end")

    def _log(self, s: str):
        ts = time.strftime("%H:%M:%S")
        self._print_line(f"[{ts}] {s}")

    def destroy(self):
        try:
            self.ble.stop()
        except Exception:
            pass
        # give BLE thread a breath
        time.sleep(0.15)
        super().destroy()

if __name__ == "__main__":
    App().mainloop()
