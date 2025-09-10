# Python Monitor for ADS1256 Load Cell Data

## Overview
Two Python scripts to monitor your Teensy ADS1256 slave device and display human-readable load cell data with statistics.

## Installation

### 1. Install Python Dependencies
```bash
pip install -r requirements.txt
```
Or manually:
```bash
pip install pyserial
```

### 2. Find Your Serial Port
**Linux/Mac:**
```bash
ls /dev/tty*
# Look for /dev/ttyACM0 or /dev/ttyUSB0
```

**Windows:**
- Check Device Manager for COM ports (e.g., COM3, COM4)

## Usage

### Simple Monitor (`simple_monitor.py`)
Basic real-time display with minimal features:

```bash
python3 simple_monitor.py
```

**Output:**
```
Sample# |    LC1    |    LC2    |    LC3    |    LC4    | SPS
---------------------------------------------------------------
      1 |  -123456  |   234567  |  -345678  |   456789  |  45.2
      2 |  -123455  |   234568  |  -345677  |   456790  |  67.8
      3 |  -123454  |   234569  |  -345676  |   456791  |  89.1
```

### Advanced Monitor (`ads1256_monitor.py`)
Full-featured monitoring with statistics and data logging:

```bash
# Basic usage
python3 ads1256_monitor.py

# Specify port
python3 ads1256_monitor.py --port /dev/ttyACM0

# Change update rate
python3 ads1256_monitor.py --update 0.5

# Save data to CSV on exit
python3 ads1256_monitor.py --save loadcell_data.csv

# All options
python3 ads1256_monitor.py --port COM3 --baudrate 115200 --update 1.0 --save data.csv
```

## Advanced Monitor Features

### Real-time Statistics Display
```
================================================================================
ADS1256 LOAD CELL MONITOR
================================================================================
Total Samples: 45,678
Runtime: 52.3 seconds
Overall SPS: 873.2
Data Rate: 10479.8 bytes/s (10.23 KB/s)
Recent SPS: 891.5
Last Update: 0.0s ago

--------------------------------------------------------------------------------
LOAD CELL VALUES (Raw ADC)
--------------------------------------------------------------------------------
LC1:  -123456 | Min:  -125000 | Max:  -120000 | Avg: -123456.7 | Std:   45.2
LC2:   234567 | Min:   230000 | Max:   240000 | Avg:  234567.1 | Std:   67.8
LC3:  -345678 | Min:  -350000 | Max:  -340000 | Avg: -345678.9 | Std:   89.1
LC4:   456789 | Min:   450000 | Max:   460000 | Avg:  456789.2 | Std:  123.4

--------------------------------------------------------------------------------
RECENT SAMPLES (Last 10)
--------------------------------------------------------------------------------
# 45669: [ -123456,   234567,  -345678,   456789]
# 45670: [ -123455,   234568,  -345677,   456790]
# 45671: [ -123454,   234569,  -345676,   456791]
...
```

### Statistics Provided
- **Sample Rate**: Current and overall samples per second
- **Data Rate**: Bytes per second and KB/s
- **Per-Channel Stats**: Min, Max, Average, Standard Deviation
- **Recent Samples**: Last 10 readings
- **Runtime**: Total monitoring time

### Data Logging
- Automatically saves data to CSV format
- Includes timestamps and sample numbers
- Can specify number of samples to save

## Command Line Options

### `ads1256_monitor.py` Options
```bash
--port, -p      Serial port (default: /dev/ttyACM0)
--baudrate, -b  Baud rate (default: 115200)
--update, -u    Update interval in seconds (default: 1.0)
--save, -s      Save data to CSV file on exit
```

### Examples
```bash
# Monitor on Windows COM port
python ads1256_monitor.py --port COM3

# Fast updates (2 times per second)
python ads1256_monitor.py --update 0.5

# Save data and use different port
python ads1256_monitor.py --port /dev/ttyUSB0 --save experiment_data.csv
```

## Data Format

### CSV Output Format
```csv
timestamp,sample_num,lc1,lc2,lc3,lc4
1634567890.123456,1,-123456,234567,-345678,456789
1634567890.124567,2,-123455,234568,-345677,456790
```

### Raw ADC Values
- **Range**: -8,388,608 to +8,388,607 (24-bit signed)
- **Resolution**: 24-bit (ADS1256 native resolution)
- **Sign**: Positive/negative depends on load direction

## Troubleshooting

### Connection Issues
```bash
# Check if device is connected
ls /dev/tty*

# Check permissions (Linux)
sudo chmod 666 /dev/ttyACM0

# Or add user to dialout group
sudo usermod -a -G dialout $USER
```

### Sync Issues
- If data appears corrupted, restart both Teensy and Python script
- Check baud rate matches (115200)
- Ensure no other programs are using the serial port

### Performance Issues
- Reduce update interval: `--update 2.0`
- Check USB cable quality
- Monitor system CPU usage

## Integration Examples

### Real-time Plotting
```python
import matplotlib.pyplot as plt
from ads1256_monitor import ADS1256Monitor

monitor = ADS1256Monitor()
monitor.connect()

# Plot data in real-time
plt.ion()
fig, ax = plt.subplots()

while True:
    if monitor.sample_buffer:
        recent_data = list(monitor.sample_buffer)[-100:]  # Last 100 samples
        lc1_data = [s['values'][0] for s in recent_data]
        
        ax.clear()
        ax.plot(lc1_data)
        ax.set_title('Load Cell 1 - Real Time')
        plt.pause(0.1)
```

### Data Processing
```python
# Convert raw ADC to engineering units
def adc_to_weight(adc_value, calibration_factor=1000.0, offset=0):
    """Convert ADC reading to weight in grams"""
    return (adc_value - offset) / calibration_factor

# Example usage
raw_value = -123456
weight_grams = adc_to_weight(raw_value, calibration_factor=1000, offset=-120000)
print(f"Weight: {weight_grams:.2f} grams")
```

This monitoring system gives you complete visibility into your load cell data with professional-grade statistics and logging capabilities!
