#!/usr/bin/env python3
"""
ESP32 ADS1256 Batch Data Collector
Collects complete batch data from ESP32 and saves all individual samples to CSV
"""

import serial as pyserial
import csv
import time
import re
import sys
from datetime import datetime

# Configuration
SERIAL_PORT = 'COM10'  # Change this to your ESP32's COM port
BAUD_RATE = 921600
COLLECTION_TIME = 10  # seconds
OUTPUT_FILE = f'ads1256_loadcell_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

def parse_batch_header(line):
    """
    Parse ESP32 batch header line
    Expected format: [ESP] Batch:42 | Samples:100 | TotalSamples:4200 | diff:1250 us
    """
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

def parse_sample_data(line):
    """
    Parse individual sample data
    Expected format: [ESP] Sample:42:0:123456:234567:345678:456789
    """
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

def is_batch_start(line):
    """Check if line indicates batch data start"""
    return '[ESP] BatchData:' in line and 'Start' in line

def is_batch_end(line):
    """Check if line indicates batch data end"""
    return '[ESP] BatchData:' in line and 'End' in line

def collect_data():
    """
    Collect complete batch data from ESP32 and save to CSV
    """
    print("="*80)
    print("ESP32 ADS1256 Batch Data Collector")
    print("="*80)
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Collection Time: {COLLECTION_TIME} seconds")
    print(f"Output File: {OUTPUT_FILE}")
    print("-" * 80)
    
    try:
        # Open serial connection
        print("Connecting to ESP32...")
        ser = pyserial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Connected successfully!")
        
        # Clear buffer
        ser.flushInput()
        
        # Prepare CSV file with headers
        csv_headers = [
            'timestamp', 'batch_id', 'sample_index', 
            'lc1', 'lc2', 'lc3', 'lc4'
        ]
        
        # Data collection variables
        start_time = time.time()
        last_print_time = start_time
        
        # Counters
        total_samples_collected = 0
        total_batches_processed = 0
        current_batch_samples = 0
        collecting_batch = False
        current_batch_id = None
        
        print(f"\nStarting data collection for {COLLECTION_TIME} seconds...")
        print("Press Ctrl+C to stop early")
        print("=" * 80)
        
        with open(OUTPUT_FILE, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=csv_headers)
            writer.writeheader()
            
            try:
                while (time.time() - start_time) < COLLECTION_TIME:
                    current_time = time.time()
                    
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            
                            # Check for batch start
                            if is_batch_start(line):
                                collecting_batch = True
                                current_batch_samples = 0
                                # Extract batch ID from start marker
                                batch_match = re.search(r'BatchData:(\d+) Start', line)
                                if batch_match:
                                    current_batch_id = int(batch_match.group(1))
                                    print(f"📦 BATCH {current_batch_id} START - collecting samples...")
                            
                            # Check for batch end
                            elif is_batch_end(line):
                                if collecting_batch and current_batch_id is not None:
                                    total_batches_processed += 1
                                    print(f"✅ BATCH {current_batch_id} END - saved {current_batch_samples} samples")
                                collecting_batch = False
                                current_batch_id = None
                                current_batch_samples = 0
                            
                            # Parse batch header (for info)
                            batch_header = parse_batch_header(line)
                            if batch_header:
                                print(f"📊 Batch {batch_header['batch_id']}: {batch_header['samples_in_batch']} samples, Total: {batch_header['total_samples']}")
                            
                            # Parse and save sample data
                            sample_data = parse_sample_data(line)
                            if sample_data and collecting_batch:
                                # Add timestamp
                                sample_data['timestamp'] = current_time
                                
                                # Write to CSV immediately
                                writer.writerow(sample_data)
                                csvfile.flush()
                                
                                total_samples_collected += 1
                                current_batch_samples += 1
                                
                                # Print every 25th sample to show progress without spam
                                if sample_data['sample_index'] % 25 == 0:
                                    print(f"💾 Sample B:{sample_data['batch_id']} S:{sample_data['sample_index']:2d} | "
                                          f"LC1:{sample_data['lc1']:8d} LC2:{sample_data['lc2']:8d} "
                                          f"LC3:{sample_data['lc3']:8d} LC4:{sample_data['lc4']:8d}")
                            
                            # Print progress every 2 seconds
                            if current_time - last_print_time >= 2.0:
                                elapsed = current_time - start_time
                                remaining = COLLECTION_TIME - elapsed
                                sample_rate = total_samples_collected / elapsed if elapsed > 0 else 0
                                batch_rate = total_batches_processed / elapsed if elapsed > 0 else 0
                                
                                print(f"\n📈 PROGRESS: {elapsed:.1f}s elapsed | "
                                      f"Batches: {total_batches_processed} ({batch_rate:.1f}/s) | "
                                      f"Samples: {total_samples_collected} ({sample_rate:.0f} SPS) | "
                                      f"Remaining: {remaining:.1f}s\n")
                                last_print_time = current_time
                            
                            # Print ESP32 stats
                            if ("ESP32 ADS1256 BATCH STATS" in line or 
                                "Sample Rate:" in line or 
                                "Batch Rate:" in line):
                                print(f"📊 {line}")
                                
                        except UnicodeDecodeError:
                            continue
                    
                    time.sleep(0.001)
                    
            except KeyboardInterrupt:
                print("\n⚠️  Collection stopped by user")
        
        # Final statistics
        end_time = time.time()
        total_time = end_time - start_time
        
        print("\n" + "="*80)
        print("📋 COLLECTION COMPLETE")
        print("="*80)
        print(f"⏱️  Total Time: {total_time:.2f} seconds")
        print(f"📦 Batches Processed: {total_batches_processed}")
        print(f"📊 Total Samples Collected: {total_samples_collected}")
        print(f"📈 Average Sample Rate: {total_samples_collected/total_time:.1f} SPS")
        print(f"📈 Average Batch Rate: {total_batches_processed/total_time:.2f} batches/s")
        print(f"💾 Data saved to: {OUTPUT_FILE}")
        
        if total_samples_collected > 0:
            print(f"\n📋 Data Summary:")
            print(f"Expected samples (1000 SPS): {int(total_time * 1000)}")
            print(f"Actual samples collected: {total_samples_collected}")
            print(f"Collection efficiency: {(total_samples_collected/(total_time*1000))*100:.1f}%")
        
        print("="*80)
        ser.close()
        return True
        
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

def main():
    """Main function"""
    if len(sys.argv) > 1:
        global SERIAL_PORT
        SERIAL_PORT = sys.argv[1]
    
    if len(sys.argv) > 2:
        global COLLECTION_TIME
        COLLECTION_TIME = int(sys.argv[2])
    
    success = collect_data()
    
    if success:
        print(f"\n✅ Data collection completed successfully!")
        print(f"📁 File saved: {OUTPUT_FILE}")
        print(f"📊 Open the CSV file in Excel or any spreadsheet application")
        print(f"💡 Each row contains one sample with all 4 load cell readings")
    else:
        print(f"\n❌ Data collection failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()
