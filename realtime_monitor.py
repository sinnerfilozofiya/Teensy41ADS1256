#!/usr/bin/env python3
"""
Real-time ADS1256 Monitor with Live Statistics
Enhanced monitoring with data rate, sample rate, and performance metrics
"""

import serial
import time
import threading
from collections import deque
import statistics
import sys
import os

class RealTimeMonitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        
        # Data storage with time-based windows
        self.samples = deque(maxlen=10000)
        self.sample_times = deque(maxlen=1000)  # For rate calculations
        self.byte_times = deque(maxlen=1000)    # For data rate calculations
        
        # Counters
        self.total_samples = 0
        self.total_bytes = 0
        self.start_time = time.time()
        
        # Current values
        self.current_lc = [0, 0, 0, 0]
        self.last_update = time.time()
        
        # Performance metrics
        self.sync_errors = 0
        self.read_errors = 0
        
        # Display settings
        self.display_mode = 'full'  # 'full', 'compact', 'minimal'
        
    def connect(self):
        """Connect to Teensy"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"✓ Connected to {self.port}")
            
            # Clear any startup messages
            time.sleep(0.5)
            self.ser.flushInput()
            
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def decode_sample(self, data):
        """Decode 12-byte sample"""
        if len(data) != 12:
            return None
        
        values = []
        for i in range(4):
            offset = i * 3
            raw = data[offset:offset+3]
            
            # 24-bit signed integer (little endian)
            value = (raw[2] << 16) | (raw[1] << 8) | raw[0]
            if value & 0x800000:  # Sign extend
                value |= 0xFF000000
                value = value - 0x100000000
            
            values.append(value)
        
        return values
    
    def calculate_rates(self):
        """Calculate current sample and data rates"""
        current_time = time.time()
        
        # Sample rate (last 1 second)
        recent_samples = [t for t in self.sample_times if current_time - t <= 1.0]
        sample_rate = len(recent_samples)
        
        # Data rate (last 1 second)
        recent_bytes = [t for t in self.byte_times if current_time - t <= 1.0]
        data_rate = len(recent_bytes) * 12  # 12 bytes per sample
        
        # Overall rates
        total_time = current_time - self.start_time
        overall_sample_rate = self.total_samples / total_time if total_time > 0 else 0
        overall_data_rate = self.total_bytes / total_time if total_time > 0 else 0
        
        return {
            'sample_rate': sample_rate,
            'data_rate': data_rate,
            'overall_sample_rate': overall_sample_rate,
            'overall_data_rate': overall_data_rate,
            'total_time': total_time
        }
    
    def calculate_channel_stats(self):
        """Calculate per-channel statistics"""
        if len(self.samples) < 2:
            return None
        
        # Use last 1000 samples for statistics
        recent_samples = list(self.samples)[-1000:]
        
        stats = []
        for ch in range(4):
            values = [sample[ch] for sample in recent_samples]
            if values:
                ch_stats = {
                    'current': values[-1],
                    'min': min(values),
                    'max': max(values),
                    'avg': statistics.mean(values),
                    'std': statistics.stdev(values) if len(values) > 1 else 0,
                    'range': max(values) - min(values)
                }
                stats.append(ch_stats)
        
        return stats
    
    def read_data_thread(self):
        """Background data reading thread"""
        buffer = b''
        
        while self.running:
            try:
                # Read available data
                new_data = self.ser.read(self.ser.in_waiting or 12)
                if new_data:
                    buffer += new_data
                    self.total_bytes += len(new_data)
                    
                    # Process complete 12-byte samples
                    while len(buffer) >= 12:
                        sample_data = buffer[:12]
                        buffer = buffer[12:]
                        
                        values = self.decode_sample(sample_data)
                        if values:
                            current_time = time.time()
                            
                            # Store sample
                            self.samples.append(values)
                            self.sample_times.append(current_time)
                            self.byte_times.append(current_time)
                            
                            self.current_lc = values
                            self.total_samples += 1
                            self.last_update = current_time
                        else:
                            self.sync_errors += 1
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning
                
            except Exception as e:
                self.read_errors += 1
                time.sleep(0.1)
    
    def clear_screen(self):
        """Clear terminal screen"""
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def display_full_stats(self):
        """Display comprehensive statistics"""
        rates = self.calculate_rates()
        ch_stats = self.calculate_channel_stats()
        
        self.clear_screen()
        
        print("=" * 100)
        print("🔬 ADS1256 REAL-TIME MONITOR")
        print("=" * 100)
        
        # Performance metrics
        print(f"📊 PERFORMANCE METRICS")
        print(f"   Sample Rate:     {rates['sample_rate']:>6.1f} SPS (current) | {rates['overall_sample_rate']:>6.1f} SPS (average)")
        print(f"   Data Rate:       {rates['data_rate']:>6.0f} B/s  (current) | {rates['overall_data_rate']:>6.0f} B/s  (average)")
        print(f"   Data Rate:       {rates['data_rate']/1024:>6.2f} KB/s (current) | {rates['overall_data_rate']/1024:>6.2f} KB/s (average)")
        print(f"   Total Samples:   {self.total_samples:,}")
        print(f"   Total Data:      {self.total_bytes:,} bytes ({self.total_bytes/1024/1024:.2f} MB)")
        print(f"   Runtime:         {rates['total_time']:.1f} seconds")
        print(f"   Sync Errors:     {self.sync_errors}")
        print(f"   Read Errors:     {self.read_errors}")
        print(f"   Last Update:     {time.time() - self.last_update:.2f}s ago")
        
        print("\n" + "=" * 100)
        print("📈 LOAD CELL DATA (Raw ADC Values)")
        print("=" * 100)
        
        if ch_stats:
            print(f"{'Channel':<8} {'Current':<10} {'Min':<10} {'Max':<10} {'Average':<12} {'Std Dev':<10} {'Range':<10}")
            print("-" * 100)
            
            for i, stats in enumerate(ch_stats):
                print(f"LC{i+1:<5} {stats['current']:<10} {stats['min']:<10} {stats['max']:<10} "
                      f"{stats['avg']:<12.1f} {stats['std']:<10.1f} {stats['range']:<10}")
        
        # Recent samples
        if len(self.samples) > 0:
            print(f"\n📋 RECENT SAMPLES (Last 5)")
            print("-" * 100)
            recent = list(self.samples)[-5:]
            for i, sample in enumerate(recent):
                sample_num = self.total_samples - len(recent) + i + 1
                values_str = " | ".join([f"{v:>8}" for v in sample])
                print(f"#{sample_num:>6}: {values_str}")
        
        print("\n" + "=" * 100)
        print("⌨️  Controls: [q]uit | [c]ompact | [m]inimal | [f]ull")
        print("=" * 100)
    
    def display_compact_stats(self):
        """Display compact statistics"""
        rates = self.calculate_rates()
        
        print(f"\r📊 SPS: {rates['sample_rate']:>5.1f} | "
              f"Data: {rates['data_rate']/1024:>5.2f} KB/s | "
              f"Samples: {self.total_samples:>6,} | "
              f"LC: [{self.current_lc[0]:>7}, {self.current_lc[1]:>7}, {self.current_lc[2]:>7}, {self.current_lc[3]:>7}]", 
              end="", flush=True)
    
    def display_minimal_stats(self):
        """Display minimal statistics"""
        rates = self.calculate_rates()
        print(f"\r{rates['sample_rate']:>5.1f} SPS | {rates['data_rate']/1024:>5.2f} KB/s | "
              f"{self.current_lc[0]:>7} {self.current_lc[1]:>7} {self.current_lc[2]:>7} {self.current_lc[3]:>7}", 
              end="", flush=True)
    
    def run_monitor(self):
        """Run the monitoring system"""
        if not self.connect():
            return
        
        self.running = True
        
        # Start data thread
        data_thread = threading.Thread(target=self.read_data_thread, daemon=True)
        data_thread.start()
        
        print("🚀 Starting monitor... Press 'q' to quit")
        time.sleep(1)
        
        try:
            while self.running:
                if self.display_mode == 'full':
                    self.display_full_stats()
                    time.sleep(1.0)
                elif self.display_mode == 'compact':
                    self.display_compact_stats()
                    time.sleep(0.1)
                elif self.display_mode == 'minimal':
                    self.display_minimal_stats()
                    time.sleep(0.1)
                
                # Check for keyboard input (simplified for cross-platform)
                # Note: For full keyboard control, run in terminal that supports it
        
        except KeyboardInterrupt:
            pass
        
        finally:
            self.running = False
            if self.ser:
                self.ser.close()
            print("\n\n✓ Monitor stopped")
    
    def save_current_data(self, filename):
        """Save current session data"""
        if not self.samples:
            print("No data to save")
            return
        
        with open(filename, 'w') as f:
            f.write("sample_num,lc1,lc2,lc3,lc4,timestamp\n")
            for i, sample in enumerate(self.samples):
                timestamp = time.time()  # Approximate
                f.write(f"{i+1},{sample[0]},{sample[1]},{sample[2]},{sample[3]},{timestamp}\n")
        
        print(f"✓ Saved {len(self.samples)} samples to {filename}")

# Import select for non-blocking input
try:
    import select
except ImportError:
    # Windows doesn't have select, use simpler approach
    import msvcrt
    
    def check_keyboard():
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8').lower()
        return None

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Real-time ADS1256 Monitor')
    parser.add_argument('--port', '-p', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Baud rate')
    parser.add_argument('--mode', '-m', choices=['full', 'compact', 'minimal'], 
                       default='full', help='Display mode')
    
    args = parser.parse_args()
    
    monitor = RealTimeMonitor(port=args.port, baudrate=args.baudrate)
    monitor.display_mode = args.mode
    
    try:
        monitor.run_monitor()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Offer to save data
        if monitor.samples:
            save = input(f"\nSave {len(monitor.samples)} samples to file? (y/n): ")
            if save.lower() == 'y':
                filename = f"realtime_data_{int(time.time())}.csv"
                monitor.save_current_data(filename)

if __name__ == "__main__":
    main()
