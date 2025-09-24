#!/usr/bin/env python3
"""
BLE Redis-like Load Cell Data Logger
Connects to ESP32 BLE slave and logs 8-channel load cell data from Redis-like system
"""

import asyncio
import struct
import csv
import time
from datetime import datetime
from bleak import BleakClient, BleakScanner
import numpy as np

# BLE Configuration (must match ESP32)
BLE_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
BLE_CHARACTERISTIC_UUID = "87654321-4321-4321-4321-cba987654321"
DEVICE_NAME = "LoadCell"

# Data structures (must match ESP32)
SAMPLES_PER_BLE_PACKET = 5
LOADCELL_SAMPLE_SIZE = 32  # 8 * int32_t = 32 bytes
BLE_PACKET_HEADER_SIZE = 1  # sample_count byte

class RedisDataLogger:
    def __init__(self, filename_prefix="redis_loadcell_data"):
        self.filename_prefix = filename_prefix
        self.csv_file = None
        self.csv_writer = None
        self.start_time = None
        self.sample_count = 0
        self.packet_count = 0
        self.bytes_received = 0
        self.last_stats_time = time.time()
        self.last_sample_count = 0
        
        # Statistics
        self.total_packets = 0
        self.total_samples = 0
        self.connection_errors = 0
        
    def create_csv_file(self):
        """Create CSV file with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.filename_prefix}_{timestamp}.csv"
        
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        header = [
            'timestamp_ms',
            'local_lc1', 'local_lc2', 'local_lc3', 'local_lc4',
            'remote_lc1', 'remote_lc2', 'remote_lc3', 'remote_lc4',
            'elapsed_ms'
        ]
        self.csv_writer.writerow(header)
        self.csv_file.flush()
        
        print(f"📁 Created CSV file: {filename}")
        return filename
        
    def parse_ble_packet(self, data):
        """Parse BLE packet containing multiple 8-channel samples"""
        if len(data) < BLE_PACKET_HEADER_SIZE:
            print(f"⚠️  Packet too short: {len(data)} bytes")
            return []
            
        # Parse header
        sample_count = data[0]
        
        # Validate packet size
        expected_size = BLE_PACKET_HEADER_SIZE + (sample_count * LOADCELL_SAMPLE_SIZE)
        if len(data) != expected_size:
            print(f"⚠️  Unexpected packet size: {len(data)}, expected {expected_size} for {sample_count} samples")
            return []
            
        samples = []
        offset = BLE_PACKET_HEADER_SIZE
        
        for i in range(sample_count):
            # Parse 8-channel sample (8 * int32_t = 32 bytes)
            sample_data = data[offset:offset + LOADCELL_SAMPLE_SIZE]
            if len(sample_data) != LOADCELL_SAMPLE_SIZE:
                print(f"⚠️  Incomplete sample {i}: {len(sample_data)} bytes")
                break
                
            # Unpack 8 int32_t values (little-endian)
            values = struct.unpack('<8i', sample_data)
            
            # Split into local and remote
            local_lc = values[0:4]   # First 4 values
            remote_lc = values[4:8]  # Last 4 values
            
            samples.append({
                'local_lc': local_lc,
                'remote_lc': remote_lc
            })
            
            offset += LOADCELL_SAMPLE_SIZE
            
        return samples
        
    def log_samples(self, samples):
        """Log samples to CSV file"""
        if not self.csv_writer:
            return
            
        current_time = time.time()
        
        for sample in samples:
            elapsed_ms = int((current_time - self.start_time) * 1000) if self.start_time else 0
            timestamp_ms = int(current_time * 1000)
            
            row = [
                timestamp_ms,
                sample['local_lc'][0], sample['local_lc'][1], sample['local_lc'][2], sample['local_lc'][3],
                sample['remote_lc'][0], sample['remote_lc'][1], sample['remote_lc'][2], sample['remote_lc'][3],
                elapsed_ms
            ]
            
            self.csv_writer.writerow(row)
            self.sample_count += 1
            
        self.csv_file.flush()
        
    def print_stats(self):
        """Print statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_stats_time
        
        if elapsed >= 5.0:  # Every 5 seconds
            samples_rate = (self.sample_count - self.last_sample_count) / elapsed
            
            print(f"📊 REDIS BLE LOGGER STATS:")
            print(f"   Samples: {self.sample_count} total | {samples_rate:.1f} samples/sec")
            print(f"   Packets: {self.packet_count} total | {self.bytes_received} bytes")
            print(f"   Target: 1000 samples/sec | Actual: {samples_rate:.1f} samples/sec")
            print(f"   Efficiency: {(samples_rate/1000)*100:.1f}%")
            print("=" * 50)
            
            self.last_stats_time = current_time
            self.last_sample_count = self.sample_count
            
    async def notification_handler(self, sender, data):
        """Handle BLE notifications"""
        self.packet_count += 1
        self.bytes_received += len(data)
        
        # Parse packet
        samples = self.parse_ble_packet(data)
        
        if samples:
            # Log to CSV
            self.log_samples(samples)
            
            # Print progress
            if self.packet_count % 100 == 0:
                avg_samples = len(samples)
                print(f"📦 Packet {self.packet_count}: {len(samples)} samples, {len(data)} bytes")
                
        # Print stats periodically
        self.print_stats()
        
    async def find_device(self):
        """Find and manually select ESP32 BLE device"""
        print(f"🔍 Scanning for BLE devices...")
        
        devices = await BleakScanner.discover(timeout=10.0)
        
        if not devices:
            print("❌ No BLE devices found")
            return None
        
        # Filter devices that have names (ignore unnamed devices)
        named_devices = [d for d in devices if d.name and d.name.strip()]
        
        if not named_devices:
            print("❌ No named BLE devices found")
            return None
        
        # Display available devices
        print(f"\n📱 Found {len(named_devices)} BLE devices:")
        print("=" * 60)
        for i, device in enumerate(named_devices):
            rssi_info = f" (RSSI: {device.rssi})" if hasattr(device, 'rssi') and device.rssi else ""
            print(f"  {i+1:2d}. {device.name:<25} | {device.address}{rssi_info}")
        print("=" * 60)
        
        # Manual selection
        while True:
            try:
                choice = input(f"\n🎯 Select device (1-{len(named_devices)}) or 'q' to quit: ").strip()
                
                if choice.lower() == 'q':
                    print("❌ User cancelled device selection")
                    return None
                
                device_index = int(choice) - 1
                
                if 0 <= device_index < len(named_devices):
                    selected_device = named_devices[device_index]
                    print(f"✅ Selected: {selected_device.name} ({selected_device.address})")
                    return selected_device.address
                else:
                    print(f"❌ Invalid choice. Please enter 1-{len(named_devices)}")
                    
            except ValueError:
                print("❌ Invalid input. Please enter a number or 'q'")
            except KeyboardInterrupt:
                print("\n❌ User cancelled")
                return None
        
    async def connect_and_log(self, duration_seconds=15):
        """Connect to BLE device and log data"""
        device_address = await self.find_device()
        if not device_address:
            return False
            
        print(f"🔗 Connecting to {device_address}...")
        
        try:
            async with BleakClient(device_address, timeout=20.0) as client:
                print(f"✅ Connected to {DEVICE_NAME}")
                
                # Create CSV file
                filename = self.create_csv_file()
                self.start_time = time.time()
                
                # Start notifications
                await client.start_notify(BLE_CHARACTERISTIC_UUID, self.notification_handler)
                print(f"📡 Started BLE notifications")
                print(f"⏱️  Logging for {duration_seconds} seconds...")
                print("=" * 50)
                
                # Log for specified duration
                await asyncio.sleep(duration_seconds)
                
                # Stop notifications
                await client.stop_notify(BLE_CHARACTERISTIC_UUID)
                print(f"🛑 Stopped notifications")
                
        except Exception as e:
            print(f"❌ Connection error: {e}")
            self.connection_errors += 1
            return False
            
        finally:
            if self.csv_file:
                self.csv_file.close()
                print(f"💾 CSV file saved: {filename}")
                
        # Final stats
        total_time = time.time() - self.start_time if self.start_time else 0
        avg_rate = self.sample_count / total_time if total_time > 0 else 0
        
        print(f"\n🎯 FINAL RESULTS:")
        print(f"   Duration: {total_time:.1f} seconds")
        print(f"   Total samples: {self.sample_count}")
        print(f"   Average rate: {avg_rate:.1f} samples/sec")
        print(f"   Target rate: 1000 samples/sec")
        print(f"   Efficiency: {(avg_rate/1000)*100:.1f}%")
        print(f"   Expected samples: {duration_seconds * 1000}")
        print(f"   Actual samples: {self.sample_count}")
        
        return True

async def main():
    """Main function"""
    print("🚀 Redis-like BLE Load Cell Data Logger")
    print("=" * 50)
    
    logger = RedisDataLogger()
    
    # Log for 15 seconds
    success = await logger.connect_and_log(duration_seconds=15)
    
    if success:
        print("✅ Logging completed successfully!")
    else:
        print("❌ Logging failed!")

if __name__ == "__main__":
    asyncio.run(main())
