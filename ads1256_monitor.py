#!/usr/bin/env python3
"""
ADS1256 Load Cell Monitor
Reads binary data from Teensy slave and displays human-readable format with statistics
"""

import serial
import struct
import time
import threading
from collections import deque
import statistics
import sys
import argparse

class ADS1256Monitor:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        
        # Data storage
        self.sample_buffer = deque(maxlen=10000)  # Store last 10k samples
        self.sample_count = 0
        self.start_time = time.time()
        
        # Statistics
        self.stats_window = deque(maxlen=1000)  # Rolling window for stats
        self.last_stats_time = time.time()
        self.bytes_received = 0
        
        # Current values
        self.current_values = [0, 0, 0, 0]
        self.last_update = time.time()
        
    def connect(self):
        """Connect to the Teensy device"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            
            # Wait for startup message
            startup_msg = self.ser.readline().decode().strip()
            if "SLAVE_MODE_READY" in startup_msg:
                print("✓ Teensy slave ready")
            else:
                print(f"Startup message: {startup_msg}")
            
            return True
        except Exception as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from device"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")
    
    def decode_sample(self, data):
        """Decode 12 bytes into 4 load cell values"""
        if len(data) != 12:
            return None
        
        values = []
        for i in range(4):
            # Extract 3 bytes for each channel
            offset = i * 3
            raw_bytes = data[offset:offset+3]
            
            # Convert to 24-bit signed integer (little endian)
            value = (raw_bytes[2] << 16) | (raw_bytes[1] << 8) | raw_bytes[0]
            
            # Sign extend from 24-bit to 32-bit
            if value & 0x800000:  # If MSB is set (negative number)
                value |= 0xFF000000
                value = value - 0x100000000  # Convert to proper negative
            
            values.append(value)
        
        return values
    
    def read_data_thread(self):
        """Background thread to continuously read data"""
        sync_attempts = 0
        max_sync_attempts = 100
        
        while self.running:
            try:
                # Read 12 bytes
                data = self.ser.read(12)
                
                if len(data) == 12:
                    values = self.decode_sample(data)
                    if values:
                        # Store sample
                        timestamp = time.time()
                        sample = {
                            'timestamp': timestamp,
                            'values': values,
                            'sample_num': self.sample_count
                        }
                        
                        self.sample_buffer.append(sample)
                        self.stats_window.append(sample)
                        self.current_values = values
                        self.sample_count += 1
                        self.bytes_received += 12
                        self.last_update = timestamp
                        
                        sync_attempts = 0  # Reset sync counter on successful read
                
                elif len(data) > 0:
                    # Partial read - try to resync
                    sync_attempts += 1
                    if sync_attempts > max_sync_attempts:
                        print("Warning: Too many sync attempts, restarting connection...")
                        break
                    
                    # Try to read remaining bytes
                    remaining = self.ser.read(12 - len(data))
                    if len(data + remaining) == 12:
                        values = self.decode_sample(data + remaining)
                        if values:
                            sync_attempts = 0
            
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.1)
    
    def calculate_statistics(self):
        """Calculate current statistics"""
        current_time = time.time()
        
        if len(self.stats_window) < 2:
            return None
        
        # Calculate sample rate
        time_span = self.stats_window[-1]['timestamp'] - self.stats_window[0]['timestamp']
        if time_span > 0:
            samples_per_second = len(self.stats_window) / time_span
        else:
            samples_per_second = 0
        
        # Calculate data rate
        data_rate = samples_per_second * 12  # 12 bytes per sample
        
        # Calculate per-channel statistics
        channel_stats = []
        for ch in range(4):
            values = [sample['values'][ch] for sample in self.stats_window]
            if values:
                ch_stats = {
                    'min': min(values),
                    'max': max(values),
                    'avg': statistics.mean(values),
                    'std': statistics.stdev(values) if len(values) > 1 else 0,
                    'current': values[-1]
                }
                channel_stats.append(ch_stats)
        
        # Overall statistics
        total_time = current_time - self.start_time
        overall_sps = self.sample_count / total_time if total_time > 0 else 0
        overall_data_rate = self.bytes_received / total_time if total_time > 0 else 0
        
        return {
            'samples_per_second': samples_per_second,
            'data_rate_bps': data_rate,
            'total_samples': self.sample_count,
            'total_time': total_time,
            'overall_sps': overall_sps,
            'overall_data_rate': overall_data_rate,
            'channel_stats': channel_stats,
            'last_update': self.last_update
        }
    
    def print_statistics(self):
        """Print current statistics to console"""
        stats = self.calculate_statistics()
        if not stats:
            print("Waiting for data...")
            return
        
        # Clear screen (works on most terminals)
        print("\033[2J\033[H", end="")
        
        print("=" * 80)
        print("ADS1256 LOAD CELL MONITOR")
        print("=" * 80)
        
        # Overall statistics
        print(f"Total Samples: {stats['total_samples']:,}")
        print(f"Runtime: {stats['total_time']:.1f} seconds")
        print(f"Overall SPS: {stats['overall_sps']:.1f}")
        print(f"Data Rate: {stats['overall_data_rate']:.1f} bytes/s ({stats['overall_data_rate']/1024:.2f} KB/s)")
        print(f"Recent SPS: {stats['samples_per_second']:.1f}")
        print(f"Last Update: {time.time() - stats['last_update']:.1f}s ago")
        
        print("\n" + "-" * 80)
        print("LOAD CELL VALUES (Raw ADC)")
        print("-" * 80)
        
        # Channel data
        for i, ch_stats in enumerate(stats['channel_stats']):
            print(f"LC{i+1}: {ch_stats['current']:>8} | "
                  f"Min: {ch_stats['min']:>8} | "
                  f"Max: {ch_stats['max']:>8} | "
                  f"Avg: {ch_stats['avg']:>8.1f} | "
                  f"Std: {ch_stats['std']:>6.1f}")
        
        print("\n" + "-" * 80)
        print("RECENT SAMPLES (Last 10)")
        print("-" * 80)
        
        # Show recent samples
        recent_samples = list(self.sample_buffer)[-10:]
        for sample in recent_samples:
            values_str = ", ".join([f"{v:>8}" for v in sample['values']])
            print(f"#{sample['sample_num']:>6}: [{values_str}]")
        
        print("\n" + "=" * 80)
        print("Press Ctrl+C to stop")
    
    def run_monitor(self, update_interval=1.0):
        """Run the monitoring loop"""
        if not self.connect():
            return
        
        self.running = True
        
        # Start data reading thread
        data_thread = threading.Thread(target=self.read_data_thread)
        data_thread.daemon = True
        data_thread.start()
        
        try:
            while self.running:
                self.print_statistics()
                time.sleep(update_interval)
        
        except KeyboardInterrupt:
            print("\nStopping monitor...")
        
        finally:
            self.disconnect()
    
    def save_data_csv(self, filename, num_samples=None):
        """Save collected data to CSV file"""
        if not self.sample_buffer:
            print("No data to save")
            return
        
        samples_to_save = list(self.sample_buffer)
        if num_samples:
            samples_to_save = samples_to_save[-num_samples:]
        
        with open(filename, 'w') as f:
            f.write("timestamp,sample_num,lc1,lc2,lc3,lc4\n")
            for sample in samples_to_save:
                f.write(f"{sample['timestamp']:.6f},{sample['sample_num']},"
                       f"{sample['values'][0]},{sample['values'][1]},"
                       f"{sample['values'][2]},{sample['values'][3]}\n")
        
        print(f"Saved {len(samples_to_save)} samples to {filename}")

def main():
    parser = argparse.ArgumentParser(description='ADS1256 Load Cell Monitor')
    parser.add_argument('--port', '-p', default='/dev/ttyACM0', 
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--update', '-u', type=float, default=1.0,
                       help='Update interval in seconds (default: 1.0)')
    parser.add_argument('--save', '-s', type=str,
                       help='Save data to CSV file on exit')
    
    args = parser.parse_args()
    
    # Create monitor
    monitor = ADS1256Monitor(port=args.port, baudrate=args.baudrate)
    
    try:
        # Run monitoring
        monitor.run_monitor(update_interval=args.update)
    
    finally:
        # Save data if requested
        if args.save:
            monitor.save_data_csv(args.save)

if __name__ == "__main__":
    main()
