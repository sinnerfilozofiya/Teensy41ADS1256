# Teensy 4.1 ADS1256 to ESP32-S3 SPI Communication

This project implements a reliable SPI data link carrying 4-channel force-plate data from a Teensy 4.1 (sampler) to an ESP32-S3 (radio). The Teensy samples 4 load cells at 1 kHz each and transmits the data in binary frames over SPI.

## Hardware Configuration

### Pin Mapping

**Teensy 4.1 (SPI1 Master)**
- SCK1 = GPIO 27
- MOSI1 = GPIO 26  
- MISO1 = GPIO 1 (or 33)
- CS1 = GPIO 0

**ESP32-S3 (SPI Slave)**
- SCLK = GPIO 38
- MOSI = GPIO 39
- MISO = GPIO 40
- CS = GPIO 41

### Connection Requirements
- 3.3V logic levels
- Short cable (<10 cm recommended)
- Common ground between devices
- Optional: 22-33Ω series resistors on SCLK/MOSI near driver if signal integrity issues occur

## Frame Structure

Fixed 144-byte frame format (little-endian, packed):

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0-1 | 2 | sync | 0xA5, 0x5A |
| 2 | 1 | plate_id | Plate identifier (set by firmware) |
| 3 | 1 | proto_ver | Protocol version (0x01) |
| 4-5 | 2 | frame_idx | Frame sequence number (wraps at 65535) |
| 6-9 | 4 | t0_us | Teensy µs timestamp of first sample |
| 10-129 | 120 | samples | 10 samples × 4 channels × 3 bytes (int24 LE) |
| 130-131 | 2 | crc16 | CRC16-CCITT-FALSE over bytes 0-129 |
| 132-143 | 12 | pad | Reserved/padding (zeros) |

### Sample Data Format
- Each sample contains 4 channels (load cells)
- Each channel is a signed 24-bit integer in little-endian format
- 10 samples per frame = 10ms of data at 1kHz sampling rate
- Sample order: [LC1, LC2, LC3, LC4] × 10 samples

### CRC16 Details
- Algorithm: CRC16-CCITT-FALSE
- Polynomial: 0x1021
- Initial value: 0xFFFF
- Calculated over bytes 0-129 (header + samples)

## Timing Model

- **Sampling Rate**: 1 kHz per channel (4 channels total)
- **Frame Rate**: 100 Hz (every 10ms)
- **SPI Speed**: 10 MHz (upgradeable to 20 MHz)
- **Transfer Time**: ~115µs per frame (144 bytes × 8 bits / 10MHz)
- **Throughput**: ~115.2 kbit/s payload data

## Software Architecture

### Teensy 4.1 Implementation
- Uses SPI0 for ADS1256 communication
- Uses SPI1 for ESP32 communication
- Maintains circular buffer of recent samples
- Transmits frames every 10ms with precise timing
- Reports statistics every 5 seconds

### ESP32-S3 Implementation
- Uses queued DMA SPI slave driver
- Pre-allocates 16 DMA-capable RX buffers
- High-priority RX task validates frames and detects gaps
- Low-priority stats task reports metrics every 5 seconds
- Never blocks SPI reception on prints or networking

## Error Detection & Recovery

### Frame Validation
1. **Sync Pattern**: Check for 0xA5, 0x5A header
2. **CRC16**: Validate data integrity
3. **Sequence Check**: Detect missed frames using frame_idx

### Statistics Tracked
- **Total**: frames_ok, frames_crc_fail, frames_missed
- **Window (5s)**: Recent error rates and timing jitter
- **Timing**: Inter-arrival time min/max, SPI transfer duration

## Test Plan

### 1. Synthetic Stream Test
- Run for 10 minutes
- Expected: win5s ok=500, crc=0, miss=0
- Expected: dt_us max ≈ 10,100µs (no gaps >20ms)

### 2. Stress Test
- Increase Serial print frequency
- Expected: ESP still shows miss=0 due to queued DMA

### 3. CRC Fault Injection
- Flip 1 bit every 200th frame on Teensy
- Expected: ESP counts crc_fail steadily, no false positives

### 4. Frequency Scaling
- Increase SPI from 10MHz to 20MHz
- Expected: No CRC errors, same latencies

### 5. Long Duration Test
- Run for 1+ hours
- Expected: No memory growth, missed==0 on stable link

## Usage Instructions

### Teensy 4.1 Setup
1. Load `ads_1256_custom_library.ino` and associated files
2. Connect ADS1256 to SPI0 pins as defined
3. Connect ESP32 to SPI1 pins as defined
4. Monitor Serial output at 115200 baud for statistics

### ESP32-S3 Setup
1. Load `esp32_spi_slave.ino`
2. Connect to Teensy SPI1 pins as defined
3. Monitor Serial output at 921600 baud for statistics
4. Optionally enable `process_load_cell_data()` for debugging

### Expected Output

**Teensy Serial Output:**
```
[T41] t=5.0s sent=500 rate=115.2 kbit/s samples=5000.0 sps xfer_us[min=115 max=120]
```

**ESP32 Serial Output:**
```
[ESP] T=5.0s | ok=500 crc=0 miss=0 | win5s ok=500 crc=0 miss=0 | rate=115.2 kb/s (5s=115.2) | sps=5000.0 (5s=5000.0) | dt_us[min=9950 max=10050] last_idx=499
```

## Troubleshooting

### Common Issues
1. **High CRC errors**: Check wiring, reduce SPI speed, add series resistors
2. **Missed frames**: Check ESP32 task priorities, increase queue size
3. **Timing jitter**: Verify Teensy isn't blocking in interrupt handlers
4. **No communication**: Verify pin connections and ground

### Debug Features
- Enable `process_load_cell_data()` in ESP32 code to see raw values
- Monitor frame_idx sequence for gaps
- Check inter-arrival timing for consistency

## Future Enhancements

### Optional Compression (proto_ver=0x02)
- First sample: absolute int24
- Next 9 samples: int16 deltas
- Overflow handling for large deltas
- Maintains 144-byte frame size

### Flow Control
- Add Teensy→ESP32 GPIO "FRAME_RDY" signal
- Trigger SPI from ESP32 interrupt
- Only needed if slave queue empties

## File Structure

```
├── ads_1256_custom_library/
│   ├── ads_1256_custom_library.ino    # Main Teensy code
│   ├── ads_1256_stuff.ino             # ADS1256 driver functions
│   └── ads1256_constants.ino          # ADS1256 register definitions
├── esp32_spi_slave.ino                # ESP32-S3 SPI slave code
└── SPI_Communication_README.md        # This documentation
```
