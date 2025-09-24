#!/usr/bin/env python3
"""
BLE Live Load Cell Data Monitor
Real-time monitoring of 8-channel load cell data from ESP32 Redis-like system
Prints raw data to console without saving to file
"""

import asyncio
import struct
import time
from datetime import datetime
from bleak import BleakClient, BleakScanner

# BLE Configuration (must match ESP32)
BLE_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
BLE_CHARACTERISTIC_UUID = "87654321-4321-4321-4321-cba987654321"
DEVICE_NAME = "LoadCell"

# Data structures (must match ESP32)
SAMPLES_PER_BLE_PACKET = 5
LOADCELL_SAMPLE_SIZE = 32  # 8 * int32_t = 32 bytes
BLE_PACKET_HEADER_SIZE = 1  # sample_count byte

class LiveMonitor:
    def __init__(self):
        self.start_time = None
        self.sample_count = 0
        self.packet_count = 0
        self.last_stats_time = time.time()
        self.last_sample_count = 0
        
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
            print(f"⚠️  Unexpected packet size: {len(data)}, expected {expected_size}")
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
            
            # Debug: Print raw values for first few samples
            if self.sample_count < 10:
                print(f"[DEBUG] Raw values: {values}")
            
            # Split into local and remote
            local_lc = values[0:4]   # First 4 values
            remote_lc = values[4:8]  # Last 4 values
            
            # Debug: Check for duplicates in Python parsing
            if self.sample_count < 10:
                if local_lc[0] == local_lc[1]:
                    print(f"[DEBUG] Python: Local LC1 == LC2 ({local_lc[0]})")
                if remote_lc[0] == remote_lc[1]:
                    print(f"[DEBUG] Python: Remote LC1 == LC2 ({remote_lc[0]})")
            
            samples.append({
                'local_lc': local_lc,
                'remote_lc': remote_lc
            })
            
            offset += LOADCELL_SAMPLE_SIZE
            
        return samples
        
    def print_live_data(self, samples):
        """Print live load cell data to console"""
        current_time = time.time()
        
        for sample in samples:
            self.sample_count += 1
            elapsed_ms = int((current_time - self.start_time) * 1000) if self.start_time else 0
            
            # Format: Sample# | Local[LC1,LC2,LC3,LC4] | Remote[LC1,LC2,LC3,LC4] | Time
            print(f"#{self.sample_count:5d} | "
                  f"L[{sample['local_lc'][0]:6d},{sample['local_lc'][1]:6d},{sample['local_lc'][2]:6d},{sample['local_lc'][3]:6d}] | "
                  f"R[{sample['remote_lc'][0]:6d},{sample['remote_lc'][1]:6d},{sample['remote_lc'][2]:6d},{sample['remote_lc'][3]:6d}] | "
                  f"{elapsed_ms:5d}ms")
        
    def print_stats(self):
        """Print periodic statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_stats_time
        
        if elapsed >= 5.0:  # Every 5 seconds
            samples_rate = (self.sample_count - self.last_sample_count) / elapsed
            
            print(f"\n📊 LIVE MONITOR STATS:")
            print(f"   Samples: {self.sample_count} total | {samples_rate:.1f} samples/sec")
            print(f"   Packets: {self.packet_count} total")
            print(f"   Target: 1000 samples/sec | Actual: {samples_rate:.1f} samples/sec")
            print(f"   Efficiency: {(samples_rate/1000)*100:.1f}%")
            print("=" * 80)
            
            self.last_stats_time = current_time
            self.last_sample_count = self.sample_count
            
    async def notification_handler(self, sender, data):
        """Handle BLE notifications"""
        self.packet_count += 1
        
        # Parse packet
        samples = self.parse_ble_packet(data)
        
        if samples:
            # Print live data
            self.print_live_data(samples)
                
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
        
    async def start_monitoring(self):
        """Start live monitoring"""
        device_address = await self.find_device()
        if not device_address:
            return False
            
        print(f"🔗 Connecting to {device_address}...")
        
        try:
            async with BleakClient(device_address, timeout=20.0) as client:
                print(f"✅ Connected to device!")
                print(f"📡 Starting live data monitoring...")
                print(f"💡 Press Ctrl+C to stop monitoring")
                print("=" * 80)
                print("Sample# | Local[LC1,  LC2,  LC3,  LC4] | Remote[LC1, LC2,  LC3,  LC4] | Time")
                print("=" * 80)
                
                self.start_time = time.time()
                
                # Start notifications
                await client.start_notify(BLE_CHARACTERISTIC_UUID, self.notification_handler)
                
                # Monitor indefinitely until Ctrl+C
                try:
                    while True:
                        await asyncio.sleep(1)
                except KeyboardInterrupt:
                    print(f"\n🛑 Monitoring stopped by user")
                
                # Stop notifications
                await client.stop_notify(BLE_CHARACTERISTIC_UUID)
                
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False
            
        # Final stats
        total_time = time.time() - self.start_time if self.start_time else 0
        avg_rate = self.sample_count / total_time if total_time > 0 else 0
        
        print(f"\n🎯 MONITORING SUMMARY:")
        print(f"   Duration: {total_time:.1f} seconds")
        print(f"   Total samples: {self.sample_count}")
        print(f"   Average rate: {avg_rate:.1f} samples/sec")
        print(f"   Target rate: 1000 samples/sec")
        print(f"   Efficiency: {(avg_rate/1000)*100:.1f}%")
        
        return True

async def main():
    """Main function"""
    print("🔴 LIVE Load Cell Data Monitor")
    print("Real-time monitoring of 8-channel load cell data")
    print("=" * 50)
    
    monitor = LiveMonitor()
    
    try:
        success = await monitor.start_monitoring()
        
        if success:
            print("✅ Monitoring completed!")
        else:
            print("❌ Monitoring failed!")
    except KeyboardInterrupt:
        print("\n❌ Monitoring interrupted by user")

if __name__ == "__main__":
    asyncio.run(main())
