# ADS1256 Slave Mode Protocol

## Overview
The Teensy is now configured as a **pure data acquisition slave** that continuously reads 4 load cells and outputs raw binary data at maximum throughput.

## Data Format

### Binary Output (12 bytes per sample)
```
[LC1_byte0][LC1_byte1][LC1_byte2][LC2_byte0][LC2_byte1][LC2_byte2][LC3_byte0][LC3_byte1][LC3_byte2][LC4_byte0][LC4_byte1][LC4_byte2]
```

- **Total size**: 12 bytes per sample
- **Format**: 3 bytes per channel (24-bit signed integer, little endian)
- **No markers**: Continuous stream for maximum efficiency
- **No text**: Pure binary data only

## Data Decoding

### Python Example (Master Device)
```python
import serial
import struct
import time

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def read_sample():
    """Read one complete sample (12 bytes)"""
    data = ser.read(12)
    if len(data) == 12:
        # Unpack 4 channels (3 bytes each, little endian, signed)
        lc1 = struct.unpack('<i', data[0:3] + b'\x00')[0]
        if lc1 & 0x800000:  # Sign extend 24-bit to 32-bit
            lc1 |= 0xFF000000
            
        lc2 = struct.unpack('<i', data[3:6] + b'\x00')[0]
        if lc2 & 0x800000:
            lc2 |= 0xFF000000
            
        lc3 = struct.unpack('<i', data[6:9] + b'\x00')[0]
        if lc3 & 0x800000:
            lc3 |= 0xFF000000
            
        lc4 = struct.unpack('<i', data[9:12] + b'\x00')[0]
        if lc4 & 0x800000:
            lc4 |= 0xFF000000
            
        return lc1, lc2, lc3, lc4
    return None

# Continuous reading
while True:
    sample = read_sample()
    if sample:
        lc1, lc2, lc3, lc4 = sample
        print(f"{lc1},{lc2},{lc3},{lc4}")
```

### Arduino/ESP32 Example (Master Device)
```cpp
void setup() {
  Serial.begin(115200);   // Debug output
  Serial1.begin(115200);  // Connection to slave Teensy
}

void loop() {
  if (Serial1.available() >= 12) {
    uint8_t data[12];
    Serial1.readBytes(data, 12);
    
    // Decode 4 channels
    int32_t lc1 = (data[2] << 16) | (data[1] << 8) | data[0];
    if (lc1 & 0x800000) lc1 |= 0xFF000000; // Sign extend
    
    int32_t lc2 = (data[5] << 16) | (data[4] << 8) | data[3];
    if (lc2 & 0x800000) lc2 |= 0xFF000000;
    
    int32_t lc3 = (data[8] << 16) | (data[7] << 8) | data[6];
    if (lc3 & 0x800000) lc3 |= 0xFF000000;
    
    int32_t lc4 = (data[11] << 16) | (data[10] << 8) | data[9];
    if (lc4 & 0x800000) lc4 |= 0xFF000000;
    
    // Send via WiFi/LoRa/etc in your preferred format
    sendWireless(lc1, lc2, lc3, lc4);
  }
}
```

## Performance Characteristics

### Throughput Optimization
- **Data size**: 12 bytes per sample (minimum possible for 4x 24-bit values)
- **No overhead**: No text, markers, or formatting
- **Continuous stream**: No gaps or delays
- **Expected rate**: ~1-2 KB/s (depending on sample rate)

### Sample Rate
- **Typical**: 800-1000 samples per second
- **Data rate**: 800 SPS × 12 bytes = 9.6 KB/s
- **Wireless ready**: Perfect for LoRa, WiFi, or cellular transmission

## Master Device Responsibilities

### Data Buffering
- Buffer incoming data to handle timing variations
- Implement frame synchronization if needed
- Handle serial buffer overruns

### Wireless Transmission
- Add your own framing/checksums for wireless
- Implement compression if needed (delta encoding, etc.)
- Handle wireless protocol specifics

### Data Processing
- Convert raw ADC values to engineering units (weight, force, etc.)
- Apply calibration factors
- Implement filtering or averaging

## Startup Sequence

1. **Power on**: Teensy initializes ADS1256
2. **Ready message**: Sends "SLAVE_MODE_READY" once
3. **Continuous data**: Immediately starts sending 12-byte samples
4. **No commands**: Slave doesn't accept any input commands

## Troubleshooting

### Sync Issues
- If data appears corrupted, restart the slave
- Master should discard partial frames on startup
- Look for consistent data patterns to verify sync

### Performance
- Use hardware serial (not USB) for best performance
- Set appropriate baud rate (115200 recommended)
- Monitor for buffer overruns on master side

## Wireless Integration Examples

### For LoRa (Low Bandwidth)
```cpp
// Send every 10th sample to reduce bandwidth
if (sampleCounter % 10 == 0) {
  LoRa.beginPacket();
  LoRa.write(data, 12);  // Send raw 12 bytes
  LoRa.endPacket();
}
```

### For WiFi/Cellular (Higher Bandwidth)
```cpp
// Buffer multiple samples and send in batches
if (bufferCount >= 50) {  // 50 samples = 600 bytes
  sendHTTPPost(buffer, bufferCount * 12);
  bufferCount = 0;
}
```

This slave configuration gives you the absolute minimum data size while maintaining full precision - perfect for your wireless transmission goals!
