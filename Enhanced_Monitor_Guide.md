# Enhanced ADS1256 Monitoring Tools

## Overview
Comprehensive monitoring and analysis tools for your ADS1256 load cell system with real-time statistics, data rate monitoring, and historical data analysis.

## Tools Available

### 1. **Real-time Monitor** (`realtime_monitor.py`)
Live monitoring with comprehensive statistics and multiple display modes.

### 2. **Data Analyzer** (`data_analyzer.py`)
Analyze collected CSV data with detailed statistics and visualizations.

### 3. **Simple Monitor** (`simple_monitor.py`)
Basic real-time display for quick testing.

## Installation

```bash
# Install all dependencies
pip install -r requirements.txt

# Or install individually
pip install pyserial pandas numpy matplotlib
```

## Real-time Monitor Usage

### Basic Usage
```bash
python3 realtime_monitor.py
```

### Advanced Options
```bash
# Specify port and display mode
python3 realtime_monitor.py --port /dev/ttyACM0 --mode full

# Compact display for continuous monitoring
python3 realtime_monitor.py --mode compact

# Minimal display for low overhead
python3 realtime_monitor.py --mode minimal
```

### Display Modes

#### **Full Mode** (Default)
Comprehensive dashboard with all statistics:
```
🔬 ADS1256 REAL-TIME MONITOR
================================================================================
📊 PERFORMANCE METRICS
   Sample Rate:      891.5 SPS (current) |  873.2 SPS (average)
   Data Rate:      10698 B/s  (current) | 10479 B/s  (average)
   Data Rate:       10.45 KB/s (current) |  10.23 KB/s (average)
   Total Samples:   45,678
   Total Data:      548,136 bytes (0.52 MB)
   Runtime:         52.3 seconds
   Sync Errors:     0
   Read Errors:     0
   Last Update:     0.02s ago

📈 LOAD CELL DATA (Raw ADC Values)
================================================================================
Channel  Current    Min        Max        Average      Std Dev    Range     
LC1      4731648    4700000    4800000    4731648.7    1234.5     100000    
LC2      -1         -50        50         -1.2         12.3       100       
LC3      -4500353   -4600000   -4400000   -4500353.9   5678.9     200000    
LC4      4622335    4600000    4700000    4622335.1    2345.6     100000    

📋 RECENT SAMPLES (Last 5)
--------------------------------------------------------------------------------
#45674:  4731648 |       -1 | -4500353 |  4622335
#45675:  4776448 |       -1 | -4296065 |  4808703
```

#### **Compact Mode**
Single line with key metrics:
```
📊 SPS:  891.5 | Data:  10.45 KB/s | Samples: 45,678 | LC: [4731648,      -1, -4500353,  4622335]
```

#### **Minimal Mode**
Ultra-compact for embedded displays:
```
891.5 SPS | 10.45 KB/s | 4731648      -1 -4500353  4622335
```

### Interactive Controls
While running, press:
- **`f`** - Switch to Full mode
- **`c`** - Switch to Compact mode  
- **`m`** - Switch to Minimal mode
- **`q`** - Quit and optionally save data

## Data Analyzer Usage

### Analyze Your Existing Data
```bash
# Basic analysis of your data.csv
python3 data_analyzer.py data.csv

# With plots and export
python3 data_analyzer.py data.csv --plot --export

# Save plots to file
python3 data_analyzer.py data.csv --save-plots
```

### Analysis Output
```
📊 ADS1256 DATA ANALYSIS REPORT
================================================================================
📁 Dataset: data.csv
📈 Total Samples: 10,001

⏱️  TIMING ANALYSIS
   Total Duration:    11.52 seconds (0.2 minutes)
   Average SPS:       868.1 samples/second
   Average Data Rate: 10417 bytes/s (10.17 KB/s)
   SPS Range:         850.2 - 890.5
   SPS Std Dev:       8.3

🏋️  LOAD CELL ANALYSIS
--------------------------------------------------------------------------------
Channel  Mean         Std Dev    Min          Max          Range        Median      
LC1      4756851.2    45123.1    4700000      4850000      150000       4755000.0   
LC2      -1.0         12.3       -50          50           100          -1.0        
LC3      -4398765.4   67890.2    -4500000     -4300000     200000       -4400000.0  
LC4      4678123.5    34567.8    4600000      4750000      150000       4680000.0   

🔍 DATA QUALITY
   LC1 Coefficient of Variation: 0.95%
   LC2 Coefficient of Variation: 1230.00%
   LC3 Coefficient of Variation: 1.54%
   LC4 Coefficient of Variation: 0.74%

📊 STABILITY ANALYSIS (First 1000 vs Last 1000 samples)
   LC1 Drift: +1234.5 (+0.03%)
   LC2 Drift: +0.1 (+10.00%)
   LC3 Drift: -567.8 (-0.01%)
   LC4 Drift: +890.2 (+0.02%)
```

## Key Metrics Explained

### **Sample Rate (SPS)**
- **Current**: Instantaneous samples per second (last 1 second)
- **Average**: Overall rate since monitoring started
- **Target**: ~800-1000 SPS for ADS1256

### **Data Rate**
- **Bytes/second**: Raw data throughput
- **KB/s**: More readable format
- **Expected**: ~10-12 KB/s for optimized binary format

### **Load Cell Statistics**
- **Mean**: Average ADC value
- **Std Dev**: Noise/stability indicator (lower is better)
- **Range**: Min to Max spread
- **CV%**: Coefficient of Variation (noise relative to signal)

### **Data Quality Indicators**
- **Sync Errors**: Lost data packets (should be 0)
- **Read Errors**: Communication issues (should be 0)
- **Drift**: Long-term stability (should be minimal)

## Performance Benchmarks

### Expected Performance
- **Sample Rate**: 800-1000 SPS
- **Data Rate**: 9.6-12 KB/s (binary mode)
- **Sync Errors**: 0
- **Stability**: <1% drift over time

### Troubleshooting Performance Issues

#### Low Sample Rate (<800 SPS)
- Check USB cable quality
- Reduce display update frequency
- Use minimal display mode
- Check system CPU usage

#### High Sync Errors
- Restart Teensy and monitor
- Check baud rate (115200)
- Verify no other programs using serial port

#### High Noise (High Std Dev)
- Check power supply quality
- Verify load cell connections
- Check grounding
- Consider hardware filtering

## Data Export and Integration

### CSV Format
```csv
timestamp,sample_num,lc1,lc2,lc3,lc4
1757512322.457088,1488,4731648,-1,-4500353,4622335
```

### Integration Examples

#### Real-time Data Streaming
```python
from realtime_monitor import RealTimeMonitor

monitor = RealTimeMonitor()
monitor.connect()

# Access live data
while True:
    if monitor.samples:
        latest = monitor.samples[-1]  # [lc1, lc2, lc3, lc4]
        print(f"Latest: {latest}")
```

#### Batch Processing
```python
from data_analyzer import DataAnalyzer

analyzer = DataAnalyzer('data.csv')
stats = analyzer.calculate_overall_stats()
print(f"Average SPS: {stats['time_stats']['avg_sample_rate']:.1f}")
```

## Wireless Integration Ready

The monitoring tools show you exactly what data rate you're achieving, making it easy to:

1. **Verify Optimization**: Confirm you're achieving target data rates
2. **Plan Wireless**: Know exact bandwidth requirements
3. **Monitor Quality**: Track data integrity and stability
4. **Debug Issues**: Identify performance bottlenecks

Your system is now fully instrumented for professional load cell monitoring! 📊🚀
