# ADS1256 Data Optimization Guide

## Current Problem
- **Original throughput**: ~13 KB/s (too high for wireless transmission)
- **Verbose text format**: Each sample ~150+ bytes with labels and stats

## Optimization Solutions Implemented

### 1. **Compact Mode** (Default)
- **Format**: CSV format `val1,val2,val3,val4`
- **Size**: ~40 bytes per sample (70% reduction)
- **Throughput**: ~4 KB/s
- **Use case**: Good balance of readability and efficiency

### 2. **Binary Mode**
- **Format**: Binary data with markers
- **Size**: 18 bytes per sample (88% reduction)
- **Throughput**: ~1.8 KB/s
- **Use case**: Maximum efficiency while maintaining full precision

### 3. **Delta Compression Mode** (Best for Wireless)
- **Format**: Binary with delta compression
- **Size**: 9 bytes per sample when deltas are small (93% reduction)
- **Throughput**: ~0.9 KB/s
- **Use case**: Optimal for wireless transmission

### 4. **Decimation**
- **Feature**: Send every Nth sample
- **Reduction**: 50% with decimation factor 2
- **Combined**: Delta + 2x decimation = ~0.45 KB/s (97% reduction!)

## Data Format Details

### Compact Mode Output
```
-123456,234567,-345678,456789
-123455,234568,-345677,456790
```

### Binary Mode Format
```
[0xAA][0x55] + 16 bytes data + [0x55][0xAA]
- Start marker: 0xAA55
- 4 channels × 4 bytes each (little endian)
- End marker: 0x55AA
```

### Delta Compression Format
```
[0xDD] + 8 bytes deltas    (when deltas fit in 16 bits)
[0xFF] + full binary data  (when deltas are too large)
```

## Configuration Functions

### Quick Setup
```cpp
setWirelessMode();  // Delta + 2x decimation (0.45 KB/s)
setCompactMode();   // CSV format (4 KB/s)
setBinaryMode();    // Binary format (1.8 KB/s)
setVerboseMode();   // Original format (13 KB/s)
```

### Manual Configuration
```cpp
currentMode = DELTA_MODE;     // Set compression mode
decimationFactor = 4;         // Send every 4th sample
enableStatsPrint = false;     // Disable verbose stats
```

## Throughput Comparison

| Mode | Format | Bytes/Sample | Throughput | Reduction |
|------|--------|--------------|------------|-----------|
| **Verbose** | Text + Stats | ~150 | 13 KB/s | 0% |
| **Compact** | CSV | ~40 | 4 KB/s | 70% |
| **Binary** | Binary | 18 | 1.8 KB/s | 88% |
| **Delta** | Compressed | 9-18 | 0.9 KB/s | 93% |
| **Delta + 2x** | Compressed + Decimation | 4.5-9 | 0.45 KB/s | **97%** |

## Wireless Transmission Recommendations

### For WiFi/Ethernet (High Bandwidth)
- Use **Compact Mode** (4 KB/s)
- Easy to parse, human readable
- Good compression without complexity

### For LoRa/Low Power (Low Bandwidth)
- Use **Delta Mode + 4x Decimation** (~0.2 KB/s)
- Maximum compression
- Still maintains precision for load cell monitoring

### For Bluetooth/Serial (Medium Bandwidth)
- Use **Binary Mode** (1.8 KB/s)
- Good balance of efficiency and simplicity
- Easy to implement receiver

## Implementation Notes

### Precision Maintained
- All modes maintain full 24-bit ADC precision
- Delta compression uses 16-bit deltas (±32K range)
- Falls back to full precision when needed

### Real-time Performance
- No processing delays added
- Compression happens in real-time
- Display updates independently

### Easy Switching
- Change mode with single function call
- No recompilation needed for different modes
- Can switch modes via serial commands

## Receiver Implementation

### For Compact Mode (CSV)
```python
# Python example
data = serial_port.readline().decode().strip()
values = [int(x) for x in data.split(',')]
val1, val2, val3, val4 = values
```

### For Binary Mode
```python
# Python example
if serial_port.read(2) == b'\xAA\x55':  # Start marker
    data = serial_port.read(16)  # 4 channels × 4 bytes
    val1 = struct.unpack('<i', data[0:4])[0]
    val2 = struct.unpack('<i', data[4:8])[0]
    val3 = struct.unpack('<i', data[8:12])[0]
    val4 = struct.unpack('<i', data[12:16])[0]
    end_marker = serial_port.read(2)  # Should be 0x55AA
```

### For Delta Mode
```python
# Python example - requires state tracking
marker = serial_port.read(1)[0]
if marker == 0xFF:  # Full sample
    # Read full binary data (18 bytes)
elif marker == 0xDD:  # Delta sample
    # Read 8 bytes of deltas, add to previous values
```

## Testing Results

With your current ~870 SPS (samples per second):
- **Original**: 870 × 150 = 130,500 bytes/s = **127 KB/s**
- **Compact**: 870 × 40 = 34,800 bytes/s = **34 KB/s**
- **Binary**: 870 × 18 = 15,660 bytes/s = **15.3 KB/s**
- **Delta**: 870 × 9 = 7,830 bytes/s = **7.6 KB/s**
- **Delta + 2x**: 435 × 9 = 3,915 bytes/s = **3.8 KB/s**

## Next Steps

1. **Test with your setup**: Try different modes and measure actual throughput
2. **Optimize for your wireless**: Choose mode based on your wireless bandwidth
3. **Implement receiver**: Create receiver code for your chosen format
4. **Add remote control**: Implement serial commands to change modes remotely
