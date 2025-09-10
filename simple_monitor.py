#!/usr/bin/env python3
"""
Simple ADS1256 Monitor
Basic real-time display of load cell data
"""

import serial
import time

def decode_sample(data):
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

def main():
    # Configuration
    PORT = '/dev/ttyACM0'  # Change this to your port (Windows: 'COM3', etc.)
    BAUDRATE = 115200
    
    try:
        # Connect to Teensy
        print(f"Connecting to {PORT}...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        
        # Wait for startup message
        startup = ser.readline().decode().strip()
        print(f"Startup: {startup}")
        
        sample_count = 0
        start_time = time.time()
        
        print("\nReading load cell data... (Press Ctrl+C to stop)")
        print("Sample# |    LC1    |    LC2    |    LC3    |    LC4    | SPS")
        print("-" * 65)
        
        while True:
            # Read 12 bytes
            data = ser.read(12)
            
            if len(data) == 12:
                values = decode_sample(data)
                if values:
                    sample_count += 1
                    
                    # Calculate samples per second
                    elapsed = time.time() - start_time
                    sps = sample_count / elapsed if elapsed > 0 else 0
                    
                    # Print formatted output
                    print(f"{sample_count:>7} | {values[0]:>8} | {values[1]:>8} | "
                          f"{values[2]:>8} | {values[3]:>8} | {sps:>5.1f}")
    
    except KeyboardInterrupt:
        print("\nStopping...")
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        if 'ser' in locals():
            ser.close()
        print("Disconnected")

if __name__ == "__main__":
    main()
