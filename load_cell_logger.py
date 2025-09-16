#!/usr/bin/env python3
"""
Load Cell Data Logger for ESP32-S3 SPI Slave
Connects to ESP32 via serial and logs raw load cell values to CSV
"""

import serial
import csv
import time
import re
import sys
from datetime import datetime
import argparse

def parse_load_cell_data(line):
    """
    Parse load cell data from ESP32 serial output
    Expected format: "DATA:frame_idx:sample_num:LC1:LC2:LC3:LC4"
    """
    # Look for compact data lines
    if line.startswith("DATA:"):
        parts = line.strip().split(':')
        if len(parts) == 7:  # DATA:frame:sample:lc1:lc2:lc3:lc4
            try:
                frame_idx = int(parts[1])
                sample_num = int(parts[2])
                lc1 = int(parts[3])
                lc2 = int(parts[4])
                lc3 = int(parts[5])
                lc4 = int(parts[6])
                return frame_idx, sample_num, lc1, lc2, lc3, lc4
            except ValueError:
                pass
    
    return None

def parse_frame_header(line):
    """
    Parse frame header information (not used with compact format)
    """
    return None

def main():
    parser = argparse.ArgumentParser(description='Log load cell data from ESP32')
    parser.add_argument('--port', '-p', default='COM3', help='Serial port (default: COM3)')
    parser.add_argument('--baud', '-b', type=int, default=921600, help='Baud rate (default: 921600)')
    parser.add_argument('--duration', '-d', type=int, default=5, help='Duration in seconds (default: 5)')
    parser.add_argument('--output', '-o', default='load_cell_data.csv', help='Output CSV file (default: load_cell_data.csv)')
    parser.add_argument('--enable-debug', action='store_true', help='Enable debug output on ESP32')
    
    args = parser.parse_args()
    
    print(f"Load Cell Data Logger")
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Duration: {args.duration} seconds")
    print(f"Output: {args.output}")
    print("-" * 50)
    
    try:
        # Open serial connection
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Connected to {args.port}")
        
        # Clear any existing data
        ser.flushInput()
        
        # If debug mode requested, send command to enable data output
        if args.enable_debug:
            print("Enabling debug mode on ESP32...")
            ser.write(b'DEBUG_ON\n')
            time.sleep(0.1)
        
        # Prepare CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"{timestamp}_{args.output}"
        
        with open(csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'frame_idx', 'sample_num', 'LC1', 'LC2', 'LC3', 'LC4']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            print(f"Logging data to {csv_filename}...")
            print("Waiting for data...")
            
            start_time = time.time()
            current_frame = None
            samples_logged = 0
            frames_logged = 0
            
            while (time.time() - start_time) < args.duration:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        # Check for compact data format: DATA:frame:sample:lc1:lc2:lc3:lc4
                        sample_data = parse_load_cell_data(line)
                        if sample_data is not None:
                            frame_idx, sample_num, lc1, lc2, lc3, lc4 = sample_data
                            
                            # Track frames
                            if current_frame != frame_idx:
                                current_frame = frame_idx
                                frames_logged += 1
                                if frames_logged % 50 == 0:
                                    print(f"Logged {frames_logged} frames, {samples_logged} samples...")
                            
                            # Write to CSV
                            writer.writerow({
                                'timestamp': time.time(),
                                'frame_idx': frame_idx,
                                'sample_num': sample_num,
                                'LC1': lc1,
                                'LC2': lc2,
                                'LC3': lc3,
                                'LC4': lc4
                            })
                            
                            samples_logged += 1
                        
                        # Print statistics and other messages
                        if '[ESP]' in line and ('ok=' in line or 'rate=' in line):
                            print(f"ESP32: {line}")
                
                except UnicodeDecodeError:
                    continue
                except KeyboardInterrupt:
                    print("\nStopping...")
                    break
        
        print(f"\nLogging complete!")
        print(f"Total frames logged: {frames_logged}")
        print(f"Total samples logged: {samples_logged}")
        print(f"Data saved to: {csv_filename}")
        
        # Calculate expected vs actual
        expected_frames = args.duration * 100  # 100 Hz frame rate
        expected_samples = expected_frames * 10  # 10 samples per frame
        
        print(f"\nExpected: {expected_frames} frames, {expected_samples} samples")
        print(f"Actual:   {frames_logged} frames, {samples_logged} samples")
        print(f"Capture rate: {(samples_logged/expected_samples)*100:.1f}%")
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        print("Make sure the ESP32 is connected and the correct port is specified.")
        sys.exit(1)
    except FileNotFoundError:
        print(f"Could not create output file: {csv_filename}")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()
