# ESP32 Radio Hop System

A robust, low-latency radio hop system for transmitting 144-byte ADC frames at 100 Hz between two ESP32-S3 devices using Wi-Fi UDP or ESP-NOW.

## System Overview

```
Teensy41 --SPI--> ESP32-S3 (TX) --WiFi/ESP-NOW--> ESP32-S3 (RX) --USB--> PC
                     |                                  |
                 SPI Slave                         Radio Receiver
                Radio Transmitter                  Frame Processor
```

### Key Features

- **Dual Transport**: Wi-Fi UDP (primary) or ESP-NOW (fallback)
- **Custom MAC Support**: Locally-administered unicast MACs for both devices
- **Low Latency**: <10ms end-to-end added latency
- **High Throughput**: 115.2 kbit/s (100 frames/s × 144 bytes)
- **Frame Integrity**: CRC16 validation and sequence tracking
- **Statistics**: Comprehensive real-time monitoring
- **Non-blocking**: SPI reception never blocked by networking

## Hardware Requirements

- 2× ESP32-S3 development boards
- 1× Teensy 4.1 (for TX side)
- Wi-Fi network (for UDP mode) or direct radio link (ESP-NOW mode)

## File Structure

```
├── common_frame.h           # Shared frame definitions and helpers
├── esp32_spi_slave.ino      # TX radio (receives from Teensy via SPI)
├── esp32_rx_radio.ino       # RX radio (receives frames over radio)
└── RADIO_HOP_README.md      # This file
```

## Configuration

### TX Radio Configuration (esp32_spi_slave.ino)

Edit the configuration section at the top of the file:

```cpp
// Transport selection (set one to 1, the other to 0)
#define USE_WIFI_UDP 1
#define USE_ESPNOW 0

// Wi-Fi Configuration
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
#define DEST_IP_STR "192.168.1.100"  // RX radio IP address
#define UDP_PORT 30303

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x02};

// Custom MAC Configuration
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_TX[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};
```

### RX Radio Configuration (esp32_rx_radio.ino)

Edit the configuration section at the top of the file:

```cpp
// Transport selection (set one to 1, the other to 0)
#define USE_WIFI_UDP 1
#define USE_ESPNOW 0

// Wi-Fi Configuration
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"
#define UDP_PORT 30303

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
static const uint8_t ESPNOW_PEER_MAC[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01};  // TX radio MAC

// Custom MAC Configuration
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_STA_MAC_RX[6] = {0x02, 0xAA, 0xBB, 0x00, 0x00, 0x02};
```

## Custom MAC Address Setup

### Rules for Custom MACs

1. **Locally Administered**: First byte bit 1 must be set (0x02, 0x06, 0x0A, etc.)
2. **Unicast**: First byte bit 0 must be clear (not 0x03, 0x07, 0x0B, etc.)
3. **Unique**: Each device must have a different MAC address
4. **Volatile**: Set at every boot (not stored in eFuses)

### Example MAC Addresses

```cpp
// TX Radio
{0x02, 0xAA, 0xBB, 0x00, 0x00, 0x01}  // 02:AA:BB:00:00:01

// RX Radio  
{0x02, 0xAA, 0xBB, 0x00, 0x00, 0x02}  // 02:AA:BB:00:00:02
```

### Verification

After setting custom MACs, the system will print the actual MAC addresses:

```
[TX] Custom STA MAC set: 02:AA:BB:00:00:01
[RX] Custom STA MAC set: 02:AA:BB:00:00:02
```

## Frame Format

### Radio Header (UDP only, 6 bytes)
```cpp
struct RadioHdr {
    uint16_t magic;     // 0xB66B (little-endian)
    uint16_t seq;       // Radio sequence number (wraps at 65535)
    uint16_t len;       // Payload length (always 144)
};
```

### Inner Frame (144 bytes, from Teensy)
```cpp
struct InnerFrame {
    uint8_t  sync[2];       // 0xA5, 0x5A
    uint8_t  plate_id;
    uint8_t  proto_ver;
    uint16_t frame_idx;     // Inner sequence number
    uint32_t t0_us;         // Timestamp
    uint8_t  samples[120];  // ADC samples (10 samples × 4 channels × 3 bytes)
    uint16_t crc16;         // CRC of frame up to this point
    uint8_t  pad[12];       // Padding to 144 bytes
};
```

### UDP Packet Structure
```
[RadioHdr: 6 bytes][InnerFrame: 144 bytes] = 150 bytes total
```

### ESP-NOW Packet Structure
```
[InnerFrame: 144 bytes] = 144 bytes total (no radio header needed)
```

## Serial Commands

Both TX and RX radios support these commands via Serial Monitor:

### Common Commands
- `DEBUG_ON` - Enable raw data output
- `DEBUG_OFF` - Disable raw data output  
- `STATUS` - Show current status and connection info

### TX Radio Commands
- `NET` - Show network transmission statistics

### RX Radio Commands
- `STATS` - Show detailed reception statistics

## Statistics Output

### TX Radio Statistics (every 5 seconds)
```
[TX] T=15.1s | SPI: ok=1500 crc=0 miss=0 | NET: sent=1500 fail=0 drops=0 | 
     win5s SPI: ok=500 crc=0 miss=0 | NET: sent=500 fail=0 drops=0 | 
     rate=115.2 kb/s (5s=115.2) | sps=4000.0 (5s=4000.0) | 
     dt_us[min=9950 max=10050] last_idx=1499
```

### RX Radio Statistics (every 5 seconds)
```
[RX] T=15.1s | ok=1500 bad_hdr=0 len_err=0 crc_err=0 missed=0 overruns=0 | 
     win5s ok=500 bad_hdr=0 len_err=0 crc_err=0 missed=0 overruns=0 | 
     rate=115.2 kb/s (5s=115.2) | sps=4000.0 (5s=4000.0) | 
     jitter_us[min=9950 max=10050] last_idx=1499
```

### Statistics Explanation

**TX Radio:**
- `SPI: ok/crc/miss` - Frames received from Teensy (good/bad CRC/missed)
- `NET: sent/fail/drops` - Network transmission (successful/failed/queue drops)
- `rate` - Data throughput in kb/s
- `sps` - Samples per second (4 channels × 10 samples/frame × frame rate)
- `dt_us` - Inter-frame timing (min/max microseconds)

**RX Radio:**
- `ok/bad_hdr/len_err/crc_err` - Received packets (good/bad header/wrong length/bad CRC)
- `missed` - Missing frames detected by inner frame_idx gaps
- `overruns` - Processing queue overflows
- `jitter_us` - Inter-arrival timing variation

## Setup Instructions

### 1. Hardware Connections

**TX Radio (ESP32-S3 connected to Teensy):**
```
Teensy 4.1    ESP32-S3
---------     --------
Pin 39   -->  GPIO 39 (MOSI)
Pin 40   -->  GPIO 40 (MISO) 
Pin 38   -->  GPIO 38 (SCLK)
Pin 41   -->  GPIO 41 (CS)
GND      -->  GND
```

**RX Radio (ESP32-S3 standalone):**
- Just power and USB connection for monitoring

### 2. Software Setup

1. **Install Arduino IDE** with ESP32 board support
2. **Copy files** to your Arduino sketches folder:
   - `common_frame.h` (must be in same folder as .ino files)
   - `esp32_spi_slave.ino` (TX radio)
   - `esp32_rx_radio.ino` (RX radio)

3. **Configure both radios** by editing the configuration sections
4. **Upload** each sketch to its respective ESP32-S3

### 3. Wi-Fi UDP Setup

1. **Configure Wi-Fi credentials** in both files
2. **Set transport mode**: `#define USE_WIFI_UDP 1` and `#define USE_ESPNOW 0`
3. **Set RX IP address** in TX radio configuration
4. **Upload and power on both devices**
5. **Check Serial Monitor** for IP addresses and connection status

### 4. ESP-NOW Setup

1. **Set transport mode**: `#define USE_WIFI_UDP 0` and `#define USE_ESPNOW 1`
2. **Configure channel** (same on both devices)
3. **Set peer MAC addresses** (TX MAC in RX config, RX MAC in TX config)
4. **Upload and power on both devices**
5. **Verify initialization** in Serial Monitor

## Testing Procedures

### 1. Basic Wi-Fi Test (10 minutes @ 100 Hz)

**Expected Results:**
- RX: `win5s ok ≈ 500`, `crc=0`, `miss=0`
- Jitter: `max ≈ 11ms` (including Wi-Fi latency)
- Custom MACs printed at startup

**Commands:**
```
# On both devices
STATUS
NET (TX only)
STATS (RX only)
```

### 2. Wi-Fi Resilience Test

**Procedure:**
1. Start normal operation
2. Power cycle Wi-Fi access point for 5 seconds
3. Observe recovery

**Expected Results:**
- TX shows `udp_drop` increases during outage
- RX shows gap in reception
- Both devices reconnect automatically
- Custom MACs persist after reconnection

### 3. Basic ESP-NOW Test (10 minutes @ 100 Hz)

**Expected Results:**
- RX: `ok ≈ 500/5s`, `missed=0`
- Send callback failures should be rare
- Single retry should recover most failures

### 4. Range Test

**Procedure:**
1. Test ESP-NOW vs Wi-Fi behavior at various distances
2. Ensure no MAC collisions with multiple powered radios

### 5. Long Duration Test (1 hour)

**Expected Results:**
- No memory growth
- `missed` stays 0 in good RF conditions
- Custom MACs printed once at startup
- Stable statistics over time

## Troubleshooting

### Common Issues

**1. Wi-Fi Connection Fails**
- Check SSID and password
- Verify Wi-Fi network is 2.4 GHz
- Check signal strength

**2. ESP-NOW Initialization Fails**
- Verify both devices use same channel
- Check peer MAC addresses are correct
- Ensure MACs are locally-administered unicast

**3. No Data Reception**
- Verify IP address configuration (Wi-Fi mode)
- Check firewall settings
- Confirm both devices are on same network

**4. High Packet Loss**
- Check Wi-Fi signal strength
- Verify network bandwidth availability
- Monitor for interference

**5. MAC Address Issues**
- Ensure first byte follows rules (0x02, 0x06, 0x0A, etc.)
- Verify each device has unique MAC
- Check that custom MAC is set before Wi-Fi/ESP-NOW init

### Debug Commands

```cpp
// Enable detailed logging
DEBUG_ON

// Check system status
STATUS

// View network statistics (TX)
NET

// View detailed reception stats (RX)  
STATS
```

### Performance Monitoring

Monitor these key metrics:

**TX Radio:**
- `NET: drops` should be 0
- `SPI: miss` should be 0
- `dt_us` should be stable around 10000 (10ms)

**RX Radio:**
- `missed` should be 0
- `overruns` should be 0
- `jitter_us max` should be reasonable (<20ms for Wi-Fi)

## Advanced Configuration

### Custom MAC Auto-Derivation

For automatic locally-administered MAC generation:

```cpp
void derive_custom_mac(uint8_t custom_mac[6]) {
    uint8_t factory_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, factory_mac);
    
    memcpy(custom_mac, factory_mac, 6);
    custom_mac[0] |= 0x02;  // Set locally administered
    custom_mac[0] &= 0xFE;  // Clear multicast (ensure unicast)
    custom_mac[5] ^= 0x01;  // Differentiate from factory
}
```

### NVS Configuration Storage

For persistent configuration storage:

```cpp
#include <nvs.h>

void save_config_to_nvs(const RadioConfig* config) {
    nvs_handle_t nvs_handle;
    nvs_open("radio_config", NVS_READWRITE, &nvs_handle);
    nvs_set_blob(nvs_handle, "config", config, sizeof(RadioConfig));
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}
```

## Performance Specifications

- **Frame Rate**: 100 Hz (10ms period)
- **Frame Size**: 144 bytes
- **Data Rate**: 115.2 kbit/s
- **Added Latency**: <10ms end-to-end
- **Transport Overhead**: 6 bytes (UDP), 0 bytes (ESP-NOW)
- **Reliability**: CRC16 + sequence number validation
- **Range**: ~100m (ESP-NOW), unlimited (Wi-Fi)

## License

This code is provided as-is for educational and development purposes.
