#!/usr/bin/env python3
"""
Ultra-Compressed ADS1256 Monitor
Monitors the new 8-byte ultra-compressed format
"""

import serial
import time
import struct

def decode_ultra_sample(data):
    """Decode 8-byte ultra-compressed sample"""
    if len(data) != 8:
        return None
    
    # Unpack 4 channels as 16-bit signed integers (little endian)
    values = struct.unpack('<hhhh', data)
    
    # Scale back up by multiplying by 256 (shift left 8 bits)
    # This restores approximate original scale
    scaled_values = [v * 256 for v in values]
    
    return scaled_values

def main():
    PORT = '/dev/ttyACM0'  # Change as needed
    BAUDRATE = 115200
    
    try:
        print(f"Connecting to {PORT}...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        
        # Wait for startup message
        startup = ser.readline().decode().strip()
        print(f"Startup: {startup}")
        
        sample_count = 0
        start_time = time.time()
        bytes_received = 0
        
        print("\nULTRA-COMPRESSED MODE (8 bytes per sample)")
        print("Sample# |    LC1    |    LC2    |    LC3    |    LC4    | SPS | Data Rate")
        print("-" * 80)
        
        while True:
            # Read 8 bytes for ultra-compressed format
            data = ser.read(8)
            
            if len(data) == 8:
                values = decode_ultra_sample(data)
                if values:
                    sample_count += 1
                    bytes_received += 8
                    
                    # Calculate rates
                    elapsed = time.time() - start_time
                    sps = sample_count / elapsed if elapsed > 0 else 0
                    data_rate = bytes_received / elapsed if elapsed > 0 else 0
                    
                    # Print formatted output
                    print(f"{sample_count:>7} | {values[0]:>8} | {values[1]:>8} | "
                          f"{values[2]:>8} | {values[3]:>8} | {sps:>5.1f} | {data_rate/1024:>5.2f} KB/s")
                    
                    # Show compression effectiveness every 1000 samples
                    if sample_count % 1000 == 0:
                        old_rate = sps * 12  # What it would be with 12-byte format
                        new_rate = sps * 8   # Current 8-byte format
                        compression = ((old_rate - new_rate) / old_rate) * 100
                        print(f">>> Compression: {compression:.1f}% reduction ({old_rate/1024:.2f} -> {new_rate/1024:.2f} KB/s)")
    
    except KeyboardInterrupt:
        print(f"\nStopping... Final stats:")
        print(f"Total samples: {sample_count:,}")
        print(f"Total data: {bytes_received:,} bytes ({bytes_received/1024:.2f} KB)")
        elapsed = time.time() - start_time
        print(f"Average SPS: {sample_count/elapsed:.1f}")
        print(f"Average data rate: {bytes_received/elapsed/1024:.2f} KB/s")
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        if 'ser' in locals():
            ser.close()
        print("Disconnected")

if __name__ == "__main__":
    main()
