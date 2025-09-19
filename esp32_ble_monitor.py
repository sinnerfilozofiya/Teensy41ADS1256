#!/usr/bin/env python3
"""
ESP32 BLE Data Monitor
High-performance data logger for ESP32 BLE sensor data transmission
Optimized for Windows with real-time monitoring and data visualization
"""

import asyncio
import struct
import time
import json
import csv
import logging
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from collections import deque
import threading
from pathlib import Path

# BLE libraries for Windows
try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic
except ImportError:
    print("Installing required packages...")
    import subprocess
    import sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "bleak"])
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic

# Optional: Install matplotlib for real-time plotting
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("Matplotlib not available - install with: pip install matplotlib numpy")
    MATPLOTLIB_AVAILABLE = False

# Optional: Install tkinter for GUI (usually comes with Python)
try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext, filedialog, messagebox
    TKINTER_AVAILABLE = True
except ImportError:
    print("Tkinter not available - GUI disabled")
    TKINTER_AVAILABLE = False

# ============================================================================
# BLE UUIDs (must match Arduino code)
# ============================================================================

SERVICE_UUID = "12345678-1234-1234-1234-123456789ABC"
DATA_CHAR_UUID = "87654321-4321-4321-4321-CBA987654321"
STATS_CHAR_UUID = "11111111-2222-3333-4444-555555555555"
CONTROL_CHAR_UUID = "AAAAAAAA-BBBB-CCCC-DDDD-EEEEEEEEEEEE"

DEVICE_NAME = "ESP32-Da"

# ============================================================================
# DATA STRUCTURES (must match Arduino structs)
# ============================================================================

@dataclass
class BLEDataPacket:
    packet_type: int
    timestamp: int
    sequence: int
    data_count: int
    reserved: int

@dataclass
class BLECombinedData:
    frame_idx: int
    sample_idx: int
    flags: int
    local_lc: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    remote_lc: List[int] = field(default_factory=lambda: [0, 0, 0, 0])

@dataclass
class BLEStatsPacket:
    packet_type: int
    timestamp: int
    local_samples_total: int
    remote_samples_total: int
    combined_samples_total: int
    local_samples_rate: int
    remote_samples_rate: int
    combined_samples_rate: int
    local_frames_total: int
    remote_frames_total: int
    buffer_usage_percent: int
    connection_quality: int
    reserved: int

# ============================================================================
# DATA PROCESSING AND STORAGE
# ============================================================================

class DataBuffer:
    """High-performance circular buffer for sensor data"""
    
    def __init__(self, max_size: int = 10000):
        self.max_size = max_size
        self.data = deque(maxlen=max_size)
        self.lock = threading.Lock()
        self.total_samples = 0
        self.start_time = time.time()
    
    def add_sample(self, sample: BLECombinedData):
        with self.lock:
            self.data.append({
                'timestamp': time.time(),
                'frame_idx': sample.frame_idx,
                'sample_idx': sample.sample_idx,
                'flags': sample.flags,
                'local_lc1': sample.local_lc[0],
                'local_lc2': sample.local_lc[1],
                'local_lc3': sample.local_lc[2],
                'local_lc4': sample.local_lc[3],
                'remote_lc1': sample.remote_lc[0],
                'remote_lc2': sample.remote_lc[1],
                'remote_lc3': sample.remote_lc[2],
                'remote_lc4': sample.remote_lc[3],
                'has_local': bool(sample.flags & 0x01),
                'has_remote': bool(sample.flags & 0x02)
            })
            self.total_samples += 1
    
    def get_latest(self, n: int = 100) -> List[Dict]:
        with self.lock:
            return list(self.data)[-n:] if len(self.data) >= n else list(self.data)
    
    def get_all(self) -> List[Dict]:
        with self.lock:
            return list(self.data)
    
    def get_rate(self) -> float:
        elapsed = time.time() - self.start_time
        return self.total_samples / elapsed if elapsed > 0 else 0
    
    def clear(self):
        with self.lock:
            self.data.clear()
            self.total_samples = 0
            self.start_time = time.time()

# ============================================================================
# BLE CLIENT CLASS
# ============================================================================

class ESP32BLEClient:
    """High-performance BLE client for ESP32 data logging"""
    
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.device_address: Optional[str] = None
        self.connected = False
        self.data_buffer = DataBuffer()
        self.stats_buffer = deque(maxlen=100)
        self.connection_callbacks = []
        self.data_callbacks = []
        self.stats_callbacks = []
        self.logger = self._setup_logger()
        
        # Statistics
        self.bytes_received = 0
        self.packets_received = 0
        self.connection_time = None
        self.last_packet_time = None
        
    def _setup_logger(self) -> logging.Logger:
        """Setup logging for debugging"""
        logger = logging.getLogger('ESP32BLE')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    async def scan_devices(self, timeout: float = 10.0) -> List[Tuple[str, str]]:
        """Scan for ESP32 devices"""
        self.logger.info("Scanning for ESP32 devices...")
        devices = []
        
        discovered = await BleakScanner.discover(timeout=timeout)
        
        for device in discovered:
            if device.name and DEVICE_NAME in device.name:
                devices.append((device.address, device.name))
                self.logger.info(f"Found ESP32: {device.name} ({device.address})")
        
        return devices
    
    async def connect(self, address: Optional[str] = None) -> bool:
        """Connect to ESP32 device"""
        try:
            if not address:
                devices = await self.scan_devices()
                if not devices:
                    self.logger.error("No ESP32 devices found")
                    return False
                address = devices[0][0]
                self.logger.info(f"Auto-connecting to {address}")
            
            self.device_address = address
            self.client = BleakClient(address)
            
            await self.client.connect()
            self.connected = True
            self.connection_time = time.time()
            
            # Subscribe to notifications
            await self.client.start_notify(DATA_CHAR_UUID, self._data_notification_handler)
            await self.client.start_notify(STATS_CHAR_UUID, self._stats_notification_handler)
            
            self.logger.info(f"Connected to ESP32 at {address}")
            self._notify_connection_callbacks(True)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            self.connected = False
            return False
    
    async def disconnect(self):
        """Disconnect from ESP32"""
        try:
            if self.client and self.connected:
                await self.client.disconnect()
                self.connected = False
                self.logger.info("Disconnected from ESP32")
                self._notify_connection_callbacks(False)
        except Exception as e:
            self.logger.error(f"Disconnect error: {e}")
    
    async def send_command(self, command: str) -> bool:
        """Send command to ESP32"""
        try:
            if not self.connected or not self.client:
                return False
            
            await self.client.write_gatt_char(CONTROL_CHAR_UUID, command.encode())
            self.logger.info(f"Sent command: {command}")
            return True
            
        except Exception as e:
            self.logger.error(f"Command send failed: {e}")
            return False
    
    def _data_notification_handler(self, sender: BleakGATTCharacteristic, data: bytearray):
        """Handle incoming data notifications"""
        try:
            self.bytes_received += len(data)
            self.packets_received += 1
            self.last_packet_time = time.time()
            
            # Parse data packet header (8 bytes)
            header = struct.unpack('<BIBB', data[:8])
            packet_type, timestamp, sequence, data_count = header
            
            if packet_type == 0:  # Combined data
                # Each sample is 38 bytes
                sample_size = 38
                offset = 8
                
                for i in range(data_count):
                    if offset + sample_size <= len(data):
                        # Parse BLECombinedData struct
                        sample_data = struct.unpack('<HBB16i', data[offset:offset + sample_size])
                        
                        frame_idx, sample_idx, flags = sample_data[0], sample_data[1], sample_data[2]
                        local_lc = list(sample_data[3:7])
                        remote_lc = list(sample_data[7:11])
                        
                        sample = BLECombinedData(
                            frame_idx=frame_idx,
                            sample_idx=sample_idx,
                            flags=flags,
                            local_lc=local_lc,
                            remote_lc=remote_lc
                        )
                        
                        self.data_buffer.add_sample(sample)
                        self._notify_data_callbacks(sample)
                        
                        offset += sample_size
            
        except Exception as e:
            self.logger.error(f"Data parsing error: {e}")
    
    def _stats_notification_handler(self, sender: BleakGATTCharacteristic, data: bytearray):
        """Handle incoming stats notifications"""
        try:
            if len(data) >= 48:  # Size of BLEStatsPacket
                stats_data = struct.unpack('<BI8IH2B', data[:48])
                
                stats = BLEStatsPacket(
                    packet_type=stats_data[0],
                    timestamp=stats_data[1],
                    local_samples_total=stats_data[2],
                    remote_samples_total=stats_data[3],
                    combined_samples_total=stats_data[4],
                    local_samples_rate=stats_data[5],
                    remote_samples_rate=stats_data[6],
                    combined_samples_rate=stats_data[7],
                    local_frames_total=stats_data[8],
                    remote_frames_total=stats_data[9],
                    buffer_usage_percent=stats_data[10],
                    connection_quality=stats_data[11],
                    reserved=stats_data[12]
                )
                
                self.stats_buffer.append(stats)
                self._notify_stats_callbacks(stats)
                
        except Exception as e:
            self.logger.error(f"Stats parsing error: {e}")
    
    def _notify_connection_callbacks(self, connected: bool):
        for callback in self.connection_callbacks:
            try:
                callback(connected)
            except Exception as e:
                self.logger.error(f"Connection callback error: {e}")
    
    def _notify_data_callbacks(self, sample: BLECombinedData):
        for callback in self.data_callbacks:
            try:
                callback(sample)
            except Exception as e:
                self.logger.error(f"Data callback error: {e}")
    
    def _notify_stats_callbacks(self, stats: BLEStatsPacket):
        for callback in self.stats_callbacks:
            try:
                callback(stats)
            except Exception as e:
                self.logger.error(f"Stats callback error: {e}")
    
    def add_connection_callback(self, callback):
        self.connection_callbacks.append(callback)
    
    def add_data_callback(self, callback):
        self.data_callbacks.append(callback)
    
    def add_stats_callback(self, callback):
        self.stats_callbacks.append(callback)

# ============================================================================
# DATA EXPORT UTILITIES
# ============================================================================

class DataExporter:
    """Export collected data to various formats"""
    
    @staticmethod
    def to_csv(data: List[Dict], filename: str):
        """Export data to CSV file"""
        if not data:
            return False
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(data)
            return True
        except Exception as e:
            print(f"CSV export error: {e}")
            return False
    
    @staticmethod
    def to_json(data: List[Dict], filename: str):
        """Export data to JSON file"""
        try:
            with open(filename, 'w') as jsonfile:
                json.dump(data, jsonfile, indent=2, default=str)
            return True
        except Exception as e:
            print(f"JSON export error: {e}")
            return False

# ============================================================================
# CONSOLE INTERFACE
# ============================================================================

class ConsoleInterface:
    """Simple console interface for monitoring"""
    
    def __init__(self, ble_client: ESP32BLEClient):
        self.ble_client = ble_client
        self.running = True
        self.last_stats_print = 0
        
        # Add callbacks
        ble_client.add_connection_callback(self._on_connection_change)
        ble_client.add_stats_callback(self._on_stats)
    
    def _on_connection_change(self, connected: bool):
        status = "CONNECTED" if connected else "DISCONNECTED"
        print(f"\n=== ESP32 {status} ===")
    
    def _on_stats(self, stats: BLEStatsPacket):
        now = time.time()
        if now - self.last_stats_print >= 2.0:  # Print every 2 seconds
            print(f"\n--- ESP32 Stats (t={stats.timestamp/1000:.1f}s) ---")
            print(f"Local:    {stats.local_samples_rate:4d} sps | {stats.local_samples_total:6d} total")
            print(f"Remote:   {stats.remote_samples_rate:4d} sps | {stats.remote_samples_total:6d} total")
            print(f"Combined: {stats.combined_samples_rate:4d} sps | {stats.combined_samples_total:6d} total")
            print(f"Frames:   L={stats.local_frames_total} R={stats.remote_frames_total}")
            print(f"Buffer:   {stats.buffer_usage_percent}% | Quality: {stats.connection_quality}%")
            print(f"Client:   {self.ble_client.data_buffer.get_rate():.1f} sps | {len(self.ble_client.data_buffer.get_all())} samples")
            
            self.last_stats_print = now
    
    async def run(self):
        """Main console loop"""
        print("=== ESP32 BLE Data Monitor ===")
        print("Commands:")
        print("  scan      - Scan for devices")
        print("  connect   - Connect to ESP32")
        print("  LOCAL_ON  - Enable local data")
        print("  LOCAL_OFF - Disable local data")
        print("  REMOTE_ON - Enable remote data")
        print("  REMOTE_OFF- Disable remote data")
        print("  BLE_ON    - Enable BLE output")
        print("  BLE_OFF   - Disable BLE output")
        print("  export    - Export data to file")
        print("  clear     - Clear data buffer")
        print("  stats     - Show statistics")
        print("  quit      - Exit")
        print()
        
        while self.running:
            try:
                command = input("ESP32-BLE> ").strip().lower()
                
                if command == 'quit':
                    self.running = False
                    
                elif command == 'scan':
                    devices = await self.ble_client.scan_devices()
                    if devices:
                        print(f"Found {len(devices)} devices:")
                        for addr, name in devices:
                            print(f"  {name} ({addr})")
                    else:
                        print("No ESP32 devices found")
                
                elif command == 'connect':
                    if await self.ble_client.connect():
                        print("Connected successfully!")
                    else:
                        print("Connection failed!")
                
                elif command.upper() in ['LOCAL_ON', 'LOCAL_OFF', 'REMOTE_ON', 'REMOTE_OFF', 'BLE_ON', 'BLE_OFF']:
                    if await self.ble_client.send_command(command.upper()):
                        print(f"Command sent: {command.upper()}")
                    else:
                        print("Command failed - not connected?")
                
                elif command == 'export':
                    data = self.ble_client.data_buffer.get_all()
                    if data:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        csv_file = f"esp32_data_{timestamp}.csv"
                        json_file = f"esp32_data_{timestamp}.json"
                        
                        if DataExporter.to_csv(data, csv_file):
                            print(f"Exported {len(data)} samples to {csv_file}")
                        if DataExporter.to_json(data, json_file):
                            print(f"Exported {len(data)} samples to {json_file}")
                    else:
                        print("No data to export")
                
                elif command == 'clear':
                    self.ble_client.data_buffer.clear()
                    print("Data buffer cleared")
                
                elif command == 'stats':
                    data = self.ble_client.data_buffer.get_all()
                    if data:
                        print(f"Samples collected: {len(data)}")
                        print(f"Collection rate: {self.ble_client.data_buffer.get_rate():.1f} samples/sec")
                        print(f"Bytes received: {self.ble_client.bytes_received}")
                        print(f"Packets received: {self.ble_client.packets_received}")
                        if self.ble_client.connection_time:
                            uptime = time.time() - self.ble_client.connection_time
                            print(f"Connection uptime: {uptime:.1f} seconds")
                    else:
                        print("No data collected yet")
                
                elif command == 'help':
                    print("Available commands: scan, connect, LOCAL_ON/OFF, REMOTE_ON/OFF, BLE_ON/OFF, export, clear, stats, quit")
                
                elif command:
                    print(f"Unknown command: {command}")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                self.running = False
            except Exception as e:
                print(f"Error: {e}")
        
        # Cleanup
        await self.ble_client.disconnect()

# ============================================================================
# GUI INTERFACE (TKINTER)
# ============================================================================

if TKINTER_AVAILABLE and MATPLOTLIB_AVAILABLE:
    class GUIInterface:
        """Advanced GUI interface with real-time plotting"""
        
        def __init__(self, ble_client: ESP32BLEClient):
            self.ble_client = ble_client
            self.root = tk.Tk()
            self.root.title("ESP32 BLE Data Monitor")
            self.root.geometry("1200x800")
            
            # Data for plotting
            self.plot_data = {
                'times': deque(maxlen=500),
                'local_lc1': deque(maxlen=500),
                'local_lc2': deque(maxlen=500),
                'local_lc3': deque(maxlen=500),
                'local_lc4': deque(maxlen=500),
                'remote_lc1': deque(maxlen=500),
                'remote_lc2': deque(maxlen=500),
                'remote_lc3': deque(maxlen=500),
                'remote_lc4': deque(maxlen=500)
            }
            
            self._setup_gui()
            self._setup_callbacks()
            
        def _setup_gui(self):
            # Create main frames
            control_frame = ttk.Frame(self.root)
            control_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
            
            data_frame = ttk.Frame(self.root)
            data_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            # Control buttons
            ttk.Button(control_frame, text="Scan Devices", command=self._scan_devices).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Connect", command=self._connect).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Disconnect", command=self._disconnect).pack(side=tk.LEFT, padx=2)
            
            ttk.Separator(control_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
            
            ttk.Button(control_frame, text="Local ON", command=lambda: self._send_command("LOCAL_ON")).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Local OFF", command=lambda: self._send_command("LOCAL_OFF")).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Remote ON", command=lambda: self._send_command("REMOTE_ON")).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Remote OFF", command=lambda: self._send_command("REMOTE_OFF")).pack(side=tk.LEFT, padx=2)
            
            ttk.Separator(control_frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
            
            ttk.Button(control_frame, text="Export CSV", command=self._export_data).pack(side=tk.LEFT, padx=2)
            ttk.Button(control_frame, text="Clear Data", command=self._clear_data).pack(side=tk.LEFT, padx=2)
            
            # Status label
            self.status_var = tk.StringVar(value="Disconnected")
            ttk.Label(control_frame, textvariable=self.status_var).pack(side=tk.RIGHT, padx=10)
            
            # Create notebook for tabs
            notebook = ttk.Notebook(data_frame)
            notebook.pack(fill=tk.BOTH, expand=True)
            
            # Real-time plot tab
            plot_frame = ttk.Frame(notebook)
            notebook.add(plot_frame, text="Real-time Plot")
            
            # Setup matplotlib plot
            self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
            self.fig.tight_layout(pad=3.0)
            
            self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
            self.canvas.draw()
            self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
            
            # Data table tab
            table_frame = ttk.Frame(notebook)
            notebook.add(table_frame, text="Data Table")
            
            # Create treeview for data display
            columns = ['Time', 'Frame', 'Sample', 'Flags', 'Local LC1-4', 'Remote LC1-4']
            self.tree = ttk.Treeview(table_frame, columns=columns, show='headings')
            
            for col in columns:
                self.tree.heading(col, text=col)
                self.tree.column(col, width=100)
            
            scrollbar = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
            self.tree.configure(yscrollcommand=scrollbar.set)
            
            self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            
            # Log tab
            log_frame = ttk.Frame(notebook)
            notebook.add(log_frame, text="Log")
            
            self.log_text = scrolledtext.ScrolledText(log_frame, height=20)
            self.log_text.pack(fill=tk.BOTH, expand=True)
            
            # Start plot animation
            self.animation = animation.FuncAnimation(self.fig, self._update_plot, interval=100, blit=False)
            
        def _setup_callbacks(self):
            self.ble_client.add_connection_callback(self._on_connection_change)
            self.ble_client.add_data_callback(self._on_data_received)
            self.ble_client.add_stats_callback(self._on_stats_received)
        
        def _on_connection_change(self, connected: bool):
            status = "Connected" if connected else "Disconnected"
            self.status_var.set(status)
            self._log(f"ESP32 {status}")
        
        def _on_data_received(self, sample: BLECombinedData):
            current_time = time.time()
            
            # Update plot data
            self.plot_data['times'].append(current_time)
            self.plot_data['local_lc1'].append(sample.local_lc[0])
            self.plot_data['local_lc2'].append(sample.local_lc[1])
            self.plot_data['local_lc3'].append(sample.local_lc[2])
            self.plot_data['local_lc4'].append(sample.local_lc[3])
            self.plot_data['remote_lc1'].append(sample.remote_lc[0])
            self.plot_data['remote_lc2'].append(sample.remote_lc[1])
            self.plot_data['remote_lc3'].append(sample.remote_lc[2])
            self.plot_data['remote_lc4'].append(sample.remote_lc[3])
            
            # Update table (only show latest 100 entries)
            if len(self.tree.get_children()) > 100:
                self.tree.delete(self.tree.get_children()[0])
            
            local_str = f"{sample.local_lc[0]},{sample.local_lc[1]},{sample.local_lc[2]},{sample.local_lc[3]}"
            remote_str = f"{sample.remote_lc[0]},{sample.remote_lc[1]},{sample.remote_lc[2]},{sample.remote_lc[3]}"
            
            self.tree.insert('', 'end', values=(
                datetime.now().strftime("%H:%M:%S.%f")[:-3],
                sample.frame_idx,
                sample.sample_idx,
                f"0x{sample.flags:02x}",
                local_str,
                remote_str
            ))
            
        def _on_stats_received(self, stats: BLEStatsPacket):
            self._log(f"Stats: Local={stats.local_samples_rate}sps Remote={stats.remote_samples_rate}sps Buffer={stats.buffer_usage_percent}%")
        
        def _update_plot(self, frame):
            if not self.plot_data['times']:
                return
                
            times = list(self.plot_data['times'])
            if not times:
                return
            
            # Convert to relative times
            base_time = times[0] if times else 0
            rel_times = [(t - base_time) for t in times]
            
            # Clear all subplots
            for ax in self.axes.flat:
                ax.clear()
            
            # Plot Local LC channels
            self.axes[0, 0].plot(rel_times, list(self.plot_data['local_lc1']), 'b-', label='LC1')
            self.axes[0, 0].plot(rel_times, list(self.plot_data['local_lc2']), 'r-', label='LC2')
            self.axes[0, 0].set_title('Local LC1-LC2')
            self.axes[0, 0].legend()
            self.axes[0, 0].grid(True)
            
            self.axes[0, 1].plot(rel_times, list(self.plot_data['local_lc3']), 'g-', label='LC3')
            self.axes[0, 1].plot(rel_times, list(self.plot_data['local_lc4']), 'm-', label='LC4')
            self.axes[0, 1].set_title('Local LC3-LC4')
            self.axes[0, 1].legend()
            self.axes[0, 1].grid(True)
            
            # Plot Remote LC channels
            self.axes[1, 0].plot(rel_times, list(self.plot_data['remote_lc1']), 'b--', label='LC1')
            self.axes[1, 0].plot(rel_times, list(self.plot_data['remote_lc2']), 'r--', label='LC2')
            self.axes[1, 0].set_title('Remote LC1-LC2')
            self.axes[1, 0].legend()
            self.axes[1, 0].grid(True)
            
            self.axes[1, 1].plot(rel_times, list(self.plot_data['remote_lc3']), 'g--', label='LC3')
            self.axes[1, 1].plot(rel_times, list(self.plot_data['remote_lc4']), 'm--', label='LC4')
            self.axes[1, 1].set_title('Remote LC3-LC4')
            self.axes[1, 1].legend()
            self.axes[1, 1].grid(True)
            
            # Set common xlabel
            for ax in self.axes[1, :]:
                ax.set_xlabel('Time (seconds)')
        
        def _log(self, message: str):
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_message = f"[{timestamp}] {message}\n"
            self.log_text.insert(tk.END, log_message)
            self.log_text.see(tk.END)
        
        def _scan_devices(self):
            asyncio.create_task(self._async_scan_devices())
        
        async def _async_scan_devices(self):
            self._log("Scanning for ESP32 devices...")
            devices = await self.ble_client.scan_devices()
            if devices:
                self._log(f"Found {len(devices)} ESP32 devices:")
                for addr, name in devices:
                    self._log(f"  {name} ({addr})")
            else:
                self._log("No ESP32 devices found")
        
        def _connect(self):
            asyncio.create_task(self._async_connect())
        
        async def _async_connect(self):
            self._log("Connecting to ESP32...")
            if await self.ble_client.connect():
                self._log("Connected successfully!")
            else:
                self._log("Connection failed!")
        
        def _disconnect(self):
            asyncio.create_task(self._async_disconnect())
        
        async def _async_disconnect(self):
            await self.ble_client.disconnect()
        
        def _send_command(self, command: str):
            asyncio.create_task(self._async_send_command(command))
        
        async def _async_send_command(self, command: str):
            if await self.ble_client.send_command(command):
                self._log(f"Command sent: {command}")
            else:
                self._log(f"Command failed: {command}")
        
        def _export_data(self):
            data = self.ble_client.data_buffer.get_all()
            if not data:
                messagebox.showwarning("No Data", "No data to export")
                return
            
            filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("JSON files", "*.json"), ("All files", "*.*")]
            )
            
            if filename:
                if filename.endswith('.json'):
                    success = DataExporter.to_json(data, filename)
                else:
                    success = DataExporter.to_csv(data, filename)
                
                if success:
                    self._log(f"Exported {len(data)} samples to {filename}")
                    messagebox.showinfo("Export Success", f"Exported {len(data)} samples to {filename}")
                else:
                    messagebox.showerror("Export Failed", "Failed to export data")
        
        def _clear_data(self):
            self.ble_client.data_buffer.clear()
            # Clear plot data
            for key in self.plot_data:
                self.plot_data[key].clear()
            # Clear table
            for item in self.tree.get_children():
                self.tree.delete(item)
            self._log("Data buffer cleared")
        
        def run(self):
            # Start the asyncio event loop in a separate thread
            def start_asyncio_loop():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_forever()
            
            asyncio_thread = threading.Thread(target=start_asyncio_loop, daemon=True)
            asyncio_thread.start()
            
            # Start the tkinter main loop
            self.root.mainloop()

# ============================================================================
# MAIN APPLICATION
# ============================================================================

async def main_console():
    """Main function for console interface"""
    ble_client = ESP32BLEClient()
    console = ConsoleInterface(ble_client)
    await console.run()

def main_gui():
    """Main function for GUI interface"""
    if not TKINTER_AVAILABLE:
        print("Tkinter not available - falling back to console mode")
        return asyncio.run(main_console())
    
    if not MATPLOTLIB_AVAILABLE:
        print("Matplotlib not available - GUI will have limited functionality")
    
    ble_client = ESP32BLEClient()
    gui = GUIInterface(ble_client)
    gui.run()

def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ESP32 BLE Data Monitor')
    parser.add_argument('--gui', action='store_true', help='Use GUI interface (default: console)')
    parser.add_argument('--device', type=str, help='ESP32 device address to connect to')
    parser.add_argument('--export', type=str, help='Auto-export data to file after collection')
    parser.add_argument('--duration', type=int, default=0, help='Collection duration in seconds (0 = infinite)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable verbose logging')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger('ESP32BLE').setLevel(logging.DEBUG)
    
    print("=== ESP32 BLE Data Monitor ===")
    print(f"Python BLE client for high-speed ESP32 sensor data")
    print(f"Tkinter GUI available: {TKINTER_AVAILABLE}")
    print(f"Matplotlib plotting available: {MATPLOTLIB_AVAILABLE}")
    print()
    
    if args.gui and TKINTER_AVAILABLE:
        print("Starting GUI interface...")
        main_gui()
    else:
        print("Starting console interface...")
        asyncio.run(main_console())

# ============================================================================
# PERFORMANCE TESTING
# ============================================================================

class PerformanceMonitor:
    """Monitor BLE performance and connection quality"""
    
    def __init__(self, ble_client: ESP32BLEClient):
        self.ble_client = ble_client
        self.start_time = time.time()
        self.sample_times = deque(maxlen=1000)
        self.packet_sizes = deque(maxlen=100)
        
        ble_client.add_data_callback(self._on_sample)
    
    def _on_sample(self, sample: BLECombinedData):
        self.sample_times.append(time.time())
    
    def get_stats(self) -> Dict:
        """Get performance statistics"""
        now = time.time()
        elapsed = now - self.start_time
        
        # Calculate sample rate over last 10 seconds
        recent_samples = [t for t in self.sample_times if now - t <= 10.0]
        recent_rate = len(recent_samples) / min(10.0, elapsed)
        
        # Calculate overall rate
        total_rate = len(self.sample_times) / elapsed if elapsed > 0 else 0
        
        return {
            'elapsed_time': elapsed,
            'total_samples': len(self.sample_times),
            'total_rate': total_rate,
            'recent_rate': recent_rate,
            'bytes_received': self.ble_client.bytes_received,
            'packets_received': self.ble_client.packets_received,
            'connection_uptime': elapsed if self.ble_client.connected else 0
        }

# ============================================================================
# INSTALLATION HELPER
# ============================================================================

def check_dependencies():
    """Check and install required dependencies"""
    required_packages = [
        'bleak',
        'matplotlib',
        'numpy'
    ]
    
    missing = []
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing.append(package)
    
    if missing:
        print(f"Missing packages: {missing}")
        print("Install with: pip install " + " ".join(missing))
        return False
    
    return True

# ============================================================================
# EXAMPLE USAGE SCRIPTS
# ============================================================================

async def example_auto_collect():
    """Example: Auto-connect and collect data for specified duration"""
    print("=== Auto Collection Example ===")
    
    ble_client = ESP32BLEClient()
    
    # Scan and connect
    print("Scanning for ESP32...")
    devices = await ble_client.scan_devices()
    if not devices:
        print("No ESP32 found!")
        return
    
    print(f"Connecting to {devices[0][1]}...")
    if not await ble_client.connect(devices[0][0]):
        print("Connection failed!")
        return
    
    # Configure for data collection
    await ble_client.send_command("LOCAL_ON")
    await ble_client.send_command("REMOTE_ON")
    await ble_client.send_command("BLE_ON")
    
    # Collect for 30 seconds
    print("Collecting data for 30 seconds...")
    await asyncio.sleep(30)
    
    # Export data
    data = ble_client.data_buffer.get_all()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"esp32_auto_collect_{timestamp}.csv"
    
    if DataExporter.to_csv(data, filename):
        print(f"Exported {len(data)} samples to {filename}")
    
    await ble_client.disconnect()
    print("Collection complete!")

if __name__ == "__main__":
    # Check if running as script vs imported module
    try:
        main()
    except KeyboardInterrupt:
        print("\nExited by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()