#!/usr/bin/env python3
"""
ESP32 ADS1256 Data Collector
Collects load cell data from ESP32 for 10 seconds and saves to CSV file
"""

import serial as pyserial
import csv
import time
import re
import sys
from datetime import datetime

# Configuration
SERIAL_PORT = 'COM10'  # Change this to your ESP32's COM port (Windows) or '/dev/ttyUSB0' (Linux)
BAUD_RATE = 921600
COLLECTION_TIME = 10  # seconds
OUTPUT_FILE = f'esp32_loadcell_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

def parse_esp32_batch_data(line):
    """
    Parse ESP32 batch data line and extract values
    Expected format: [ESP] Batch:42 | Samples:100 | TotalSamples:4200 | diff:1250 us
    """
    # Regular expression to match the ESP32 batch format
    pattern = r'\[ESP\] Batch:(\d+) \| Samples:(\d+) \| TotalSamples:(\d+) \| diff:(-?\d+) us'
    
    match = re.search(pattern, line)
    if match:
        return {
            'timestamp': time.time(),
            'batch_id': int(match.group(1)),
            'samples_in_batch': int(match.group(2)),
            'total_samples': int(match.group(3)),
            'time_diff_us': int(match.group(4))
        }
    return None

def parse_esp32_sample_data(line):
    """
    Parse ESP32 individual sample data
    Expected format: [ESP] Sample:42:0:123456:234567:345678:456789
    Format: [ESP] Sample:batch_id:sample_index:lc1:lc2:lc3:lc4
    """
    # Regular expression to match the new sample format
    pattern = r'\[ESP\] Sample:(\d+):(\d+):(-?\d+):(-?\d+):(-?\d+):(-?\d+)'
    
    match = re.search(pattern, line)
    if match:
        return {
            'timestamp': time.time(),
            'batch_id': int(match.group(1)),
            'sample_index': int(match.group(2)),
            'lc1': int(match.group(3)),
            'lc2': int(match.group(4)),
            'lc3': int(match.group(5)),
            'lc4': int(match.group(6))
        }
    return None

def parse_batch_markers(line):
    """
    Parse batch start/end markers
    Expected formats: 
    [ESP] BatchData:42 Start
    [ESP] BatchData:42 End
    """
    start_pattern = r'\[ESP\] BatchData:(\d+) Start'
    end_pattern = r'\[ESP\] BatchData:(\d+) End'
    
    start_match = re.search(start_pattern, line)
    if start_match:
        return {'type': 'batch_start', 'batch_id': int(start_match.group(1))}
    
    end_match = re.search(end_pattern, line)
    if end_match:
        return {'type': 'batch_end', 'batch_id': int(end_match.group(1))}
    
    return None

def collect_data():
    """
    Collect data from ESP32 for specified time and save to CSV
    """
    print(f"ESP32 ADS1256 Data Collector")
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"Collection Time: {COLLECTION_TIME} seconds")
    print(f"Output File: {OUTPUT_FILE}")
    print("-" * 50)
    
    try:
        # Open serial connection
        print("Connecting to ESP32...")
        ser = pyserial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("Connected successfully!")
        
        # Clear any existing data in buffer
        ser.flushInput()
        
        # Prepare CSV files for different data types
        batch_csv_headers = [
            'timestamp', 'batch_id', 'samples_in_batch', 'total_samples', 'time_diff_us'
        ]
        
        sample_csv_headers = [
            'timestamp', 'batch_id', 'sample_index', 'lc1', 'lc2', 'lc3', 'lc4'
        ]
        
        batch_data_points = []
        sample_data_points = []
        start_time = time.time()
        last_print_time = start_time
        batch_count = 0
        sample_count = 0
        current_batch_id = None
        collecting_batch_data = False
        samples_in_current_batch = 0
        
        print(f"\nStarting data collection...")
        print("Press Ctrl+C to stop early")
        
        # Create separate CSV files for batch data and sample data
        batch_file = OUTPUT_FILE.replace('.csv', '_batches.csv')
        sample_file = OUTPUT_FILE.replace('.csv', '_samples.csv')
        
        with open(batch_file, 'w', newline='') as batch_csvfile, \
             open(sample_file, 'w', newline='') as sample_csvfile:
            
            batch_writer = csv.DictWriter(batch_csvfile, fieldnames=batch_csv_headers)
            sample_writer = csv.DictWriter(sample_csvfile, fieldnames=sample_csv_headers)
            batch_writer.writeheader()
            sample_writer.writeheader()
            
            try:
                while (time.time() - start_time) < COLLECTION_TIME:
                    current_time = time.time()
                    
                    # Read line from serial
                    if ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            
                            # Parse batch markers
                            marker = parse_batch_markers(line)
                            if marker:
                                if marker['type'] == 'batch_start':
                                    collecting_batch_data = True
                                    current_batch_id = marker['batch_id']
                                    samples_in_current_batch = 0
                                    print(f"[BATCH START] Batch {current_batch_id} - collecting samples...")
                                elif marker['type'] == 'batch_end':
                                    collecting_batch_data = False
                                    print(f"[BATCH END] Batch {current_batch_id} - collected {samples_in_current_batch} samples")
                                    current_batch_id = None
                            
                            # Parse ESP32 batch summary data
                            batch_data = parse_esp32_batch_data(line)
                            if batch_data:
                                # Write to batch CSV immediately
                                batch_writer.writerow(batch_data)
                                batch_csvfile.flush()
                                
                                batch_data_points.append(batch_data)
                                batch_count += 1
                                
                                # Print real-time batch info
                                print(f"[BATCH] {batch_data['batch_id']:4d} | Samples: {batch_data['samples_in_batch']:3d} | Total: {batch_data['total_samples']:6d} | Diff: {batch_data['time_diff_us']:8d} us")
                            
                            # Parse ESP32 individual sample data
                            sample_data = parse_esp32_sample_data(line)
                            if sample_data and collecting_batch_data:
                                # Write to sample CSV immediately
                                sample_writer.writerow(sample_data)
                                sample_csvfile.flush()
                                
                                sample_data_points.append(sample_data)
                                sample_count += 1
                                samples_in_current_batch += 1
                                
                                # Print every 10th sample to avoid spam
                                if sample_data['sample_index'] % 10 == 0:
                                    print(f"[SAMPLE] B:{sample_data['batch_id']} S:{sample_data['sample_index']:2d} | LC1:{sample_data['lc1']:8d} LC2:{sample_data['lc2']:8d} LC3:{sample_data['lc3']:8d} LC4:{sample_data['lc4']:8d}")
                            
                            # Print progress every second
                            if current_time - last_print_time >= 1.0:
                                elapsed = current_time - start_time
                                remaining = COLLECTION_TIME - elapsed
                                batch_rate = batch_count / elapsed if elapsed > 0 else 0
                                sample_rate = sample_count / elapsed if elapsed > 0 else 0
                                
                                print(f"\n[PROGRESS] Time: {elapsed:.1f}s | Batches: {batch_count} ({batch_rate:.1f}/s) | Samples: {sample_count} ({sample_rate:.1f}/s) | Remaining: {remaining:.1f}s\n")
                                last_print_time = current_time
                            
                            # Also print stats lines for monitoring
                            if "ESP32 ADS1256 BATCH STATS" in line or "Runtime:" in line or "Sample Rate:" in line or "Batch Rate:" in line:
                                print(f"[STATS] {line}")
                                
                        except UnicodeDecodeError:
                            continue  # Skip corrupted lines
                    
                    time.sleep(0.001)  # Small delay to prevent excessive CPU usage
                    
            except KeyboardInterrupt:
                print("\nCollection stopped by user")
        
        # Final statistics
        end_time = time.time()
        total_time = end_time - start_time
        
        print("\n" + "="*80)
        print("COLLECTION COMPLETE")
        print("="*80)
        print(f"Total Time: {total_time:.2f} seconds")
        print(f"Total Batches: {batch_count}")
        print(f"Total Sample Previews: {sample_count}")
        print(f"Average Batch Rate: {batch_count/total_time:.2f} batches/s")
        print(f"Average Sample Preview Rate: {sample_count/total_time:.2f} samples/s")
        print(f"Estimated Total Samples: {batch_count * 100} (assuming 100 samples per batch)")
        print(f"Batch data saved to: {batch_file}")
        print(f"Sample data saved to: {sample_file}")
        
        if batch_data_points:
            # Calculate batch statistics
            first_batch = batch_data_points[0]
            last_batch = batch_data_points[-1]
            
            print(f"\nBatch Summary:")
            print(f"Batch Range: {first_batch['batch_id']} to {last_batch['batch_id']}")
            print(f"Total Samples Reported: {last_batch['total_samples']}")
            
            # Calculate timing statistics
            time_diffs = [d['time_diff_us'] for d in batch_data_points]
            print(f"Time Diff Range: {min(time_diffs)} to {max(time_diffs)} us")
            print(f"Average Time Diff: {sum(time_diffs)/len(time_diffs):.0f} us")
        
        if sample_data_points:
            print(f"\nSample Preview Summary:")
            print(f"Sample Previews Collected: {len(sample_data_points)}")
            
            # Calculate load cell value ranges from sample previews
            print(f"\nLoad Cell Value Ranges (from previews):")
            print(f"LC1: {min(d['lc1'] for d in sample_data_points)} to {max(d['lc1'] for d in sample_data_points)}")
            print(f"LC2: {min(d['lc2'] for d in sample_data_points)} to {max(d['lc2'] for d in sample_data_points)}")
            print(f"LC3: {min(d['lc3'] for d in sample_data_points)} to {max(d['lc3'] for d in sample_data_points)}")
            print(f"LC4: {min(d['lc4'] for d in sample_data_points)} to {max(d['lc4'] for d in sample_data_points)}")
        
        ser.close()
        
    except Exception as e:
        if "SerialException" in str(type(e)) or "could not open port" in str(e):
            print(f"Serial connection error: {e}")
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
            print(f"Error: {e}")
        return False
    
    return True

def main():
    """
    Main function
    """
    if len(sys.argv) > 1:
        global SERIAL_PORT
        SERIAL_PORT = sys.argv[1]
    
    if len(sys.argv) > 2:
        global COLLECTION_TIME
        COLLECTION_TIME = int(sys.argv[2])
    
    success = collect_data()
    
    if success:
        print(f"\n✅ Data collection completed successfully!")
        print(f"📁 Batch data: {OUTPUT_FILE.replace('.csv', '_batches.csv')}")
        print(f"📁 Sample data: {OUTPUT_FILE.replace('.csv', '_samples.csv')}")
        print(f"📊 You can now analyze the data in Excel or any CSV viewer")
        print(f"💡 Batch file shows timing and throughput info")
        print(f"💡 Sample file shows actual load cell values from previews")
    else:
        print(f"\n❌ Data collection failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()
