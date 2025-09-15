#!/usr/bin/env python3
"""
ESP32 ADS1256 Real-time Monitor
Shows live load cell data from ESP32 batch protocol in terminal
"""

import serial as pyserial
import time
import re
import sys
from datetime import datetime

# Configuration
SERIAL_PORT = 'COM10'  # Change this to your ESP32's COM port
BAUD_RATE = 921600

def parse_esp32_batch_data(line):
    """Parse ESP32 batch data line"""
    pattern = r'\[ESP\] Batch:(\d+) \| Samples:(\d+) \| TotalSamples:(\d+) \| diff:(-?\d+) us'
    match = re.search(pattern, line)
    if match:
        return {
            'batch_id': int(match.group(1)),
            'samples_in_batch': int(match.group(2)),
            'total_samples': int(match.group(3)),
            'time_diff_us': int(match.group(4))
        }
    return None

def parse_esp32_sample_data(line):
    """Parse ESP32 individual sample data"""
    pattern = r'\[ESP\] Sample:(\d+):(\d+):(-?\d+):(-?\d+):(-?\d+):(-?\d+)'
    match = re.search(pattern, line)
    if match:
        return {
            'batch_id': int(match.group(1)),
            'sample_index': int(match.group(2)),
            'lc1': int(match.group(3)),
            'lc2': int(match.group(4)),
            'lc3': int(match.group(5)),
            'lc4': int(match.group(6))
        }
    return None

def parse_batch_markers(line):
    """Parse batch start/end markers"""
    start_pattern = r'\[ESP\] BatchData:(\d+) Start'
    end_pattern = r'\[ESP\] BatchData:(\d+) End'
    
    start_match = re.search(start_pattern, line)
    if start_match:
        return {'type': 'batch_start', 'batch_id': int(start_match.group(1))}
    
    end_match = re.search(end_pattern, line)
    if end_match:
        return {'type': 'batch_end', 'batch_id': int(end_match.group(1))}
    
    return None

def format_large_number(num):
    """Format large numbers with commas for readability"""
    return f"{num:,}"

def monitor_realtime():
    """Monitor ESP32 data in real-time"""
    print("="*80)
    print("ESP32 ADS1256 Real-time Monitor")
    print("="*80)
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print("Press Ctrl+C to stop")
    print("-" * 80)
    
    try:
        # Open serial connection
        print("Connecting to ESP32...")
        ser = pyserial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Connected successfully!\n")
        
        # Clear buffer
        ser.flushInput()
        
        start_time = time.time()
        batch_count = 0
        sample_count = 0
        last_stats_time = start_time
        collecting_batch_data = False
        samples_in_current_batch = 0
        
        print("Waiting for data...")
        print("=" * 80)
        
        try:
            while True:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        current_time = time.time()
                        
                        # Parse batch markers
                        marker = parse_batch_markers(line)
                        if marker:
                            if marker['type'] == 'batch_start':
                                collecting_batch_data = True
                                samples_in_current_batch = 0
                                print(f"🚀 BATCH {marker['batch_id']} START - collecting samples...")
                            elif marker['type'] == 'batch_end':
                                collecting_batch_data = False
                                print(f"✅ BATCH {marker['batch_id']} END - collected {samples_in_current_batch} samples\n")
                        
                        # Parse batch summary data
                        batch_data = parse_esp32_batch_data(line)
                        if batch_data:
                            batch_count += 1
                            elapsed = current_time - start_time
                            batch_rate = batch_count / elapsed if elapsed > 0 else 0
                            estimated_sps = batch_rate * 100  # 100 samples per batch
                            
                            print(f"🔄 BATCH {batch_data['batch_id']:4d} | "
                                  f"Samples: {batch_data['samples_in_batch']:3d} | "
                                  f"Total: {format_large_number(batch_data['total_samples']):>8s} | "
                                  f"Rate: {batch_rate:5.1f} B/s | "
                                  f"Est SPS: {estimated_sps:6.0f} | "
                                  f"Diff: {batch_data['time_diff_us']:8d} μs")
                        
                        # Parse individual sample data
                        sample_data = parse_esp32_sample_data(line)
                        if sample_data and collecting_batch_data:
                            sample_count += 1
                            samples_in_current_batch += 1
                            
                            # Show every 20th sample to avoid spam
                            if sample_data['sample_index'] % 20 == 0:
                                print(f"📊 SAMPLE B:{sample_data['batch_id']} S:{sample_data['sample_index']:2d} | "
                                      f"LC1: {sample_data['lc1']:8d} | "
                                      f"LC2: {sample_data['lc2']:8d} | "
                                      f"LC3: {sample_data['lc3']:8d} | "
                                      f"LC4: {sample_data['lc4']:8d}")
                        
                        # Print ESP32 stats when they appear
                        if ("ESP32 ADS1256 BATCH STATS" in line or 
                            "Sample Rate:" in line or 
                            "Batch Rate:" in line or
                            "Total Samples Received:" in line):
                            print(f"📈 {line}")
                        
                        # Print periodic summary
                        if current_time - last_stats_time >= 5.0:
                            elapsed = current_time - start_time
                            print(f"\n{'='*80}")
                            print(f"⏱️  MONITOR STATS | Runtime: {elapsed:.1f}s | "
                                  f"Batches: {batch_count} | Samples: {sample_count}")
                            if batch_count > 0:
                                avg_batch_rate = batch_count / elapsed
                                est_total_samples = batch_count * 100
                                print(f"📊 Avg Batch Rate: {avg_batch_rate:.1f}/s | "
                                      f"Est Total Samples: {format_large_number(est_total_samples)} | "
                                      f"Est Sample Rate: {est_total_samples/elapsed:.0f} SPS")
                            print(f"{'='*80}\n")
                            last_stats_time = current_time
                            
                    except UnicodeDecodeError:
                        continue
                
                time.sleep(0.001)  # Small delay
                
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")
            
        # Final summary
        end_time = time.time()
        total_time = end_time - start_time
        
        print("\n" + "="*80)
        print("MONITORING SUMMARY")
        print("="*80)
        print(f"Total Runtime: {total_time:.2f} seconds")
        print(f"Batches Received: {batch_count}")
        print(f"Sample Previews: {sample_count}")
        if batch_count > 0:
            print(f"Average Batch Rate: {batch_count/total_time:.2f} batches/s")
            print(f"Estimated Total Samples: {format_large_number(batch_count * 100)}")
            print(f"Estimated Sample Rate: {(batch_count * 100)/total_time:.0f} SPS")
        print("="*80)
        
        ser.close()
        
    except Exception as e:
        if "SerialException" in str(type(e)) or "could not open port" in str(e):
            print(f"❌ Serial connection error: {e}")
            print(f"Make sure ESP32 is connected to {SERIAL_PORT}")
            print("Available ports might be:")
            try:
                from serial.tools import list_ports
                ports = list_ports.comports()
                for port in ports:
                    print(f"  {port.device} - {port.description}")
            except:
                print("  Could not list available ports")
        else:
            print(f"❌ Error: {e}")
        return False
    
    return True

def main():
    """Main function"""
    if len(sys.argv) > 1:
        global SERIAL_PORT
        SERIAL_PORT = sys.argv[1]
    
    success = monitor_realtime()
    
    if not success:
        print(f"\n❌ Monitoring failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()
