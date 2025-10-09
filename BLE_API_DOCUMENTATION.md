# ESP32 Load Cell BLE Server - API Documentation

## Overview

The ESP32 BLE Server provides real-time load cell data from 8 channels (4 local + 4 remote) over Bluetooth Low Energy. It supports bidirectional communication: streaming sensor data via notifications and receiving control commands via write operations.

**Device Name:** `LoadCell_BLE_Server`

---

## Table of Contents

1. [BLE Service & Characteristics](#ble-service--characteristics)
2. [Connection Guide](#connection-guide)
3. [Data Streaming (Notifications)](#data-streaming-notifications)
4. [Command Interface (Write)](#command-interface-write)
5. [Data Format Specification](#data-format-specification)
6. [Command Reference](#command-reference)
7. [Client Implementation Examples](#client-implementation-examples)
8. [Troubleshooting](#troubleshooting)

---

## BLE Service & Characteristics

### Service UUID
```
12345678-1234-1234-1234-123456789abc
```

### Characteristics

#### 1. Data Characteristic (Notifications)
- **UUID:** `87654321-4321-4321-4321-cba987654321`
- **Properties:** `READ` + `NOTIFY`
- **Purpose:** Streams load cell data packets to connected clients
- **Data Rate:** ~100 packets/second
- **Packet Size:** 1-161 bytes (variable, typically 161 bytes)

#### 2. Command Characteristic (Write)
- **UUID:** `11111111-2222-3333-4444-555555555555`
- **Properties:** `WRITE` + `WRITE_NO_RESPONSE`
- **Purpose:** Receives control commands from clients
- **Format:** UTF-8 string (case-insensitive)
- **Max Length:** ~100 characters

---

## Connection Guide

### Connection Steps

1. **Scan for BLE Devices**
   - Look for device name: `LoadCell_BLE_Server`
   - Filter by service UUID: `12345678-1234-1234-1234-123456789abc`

2. **Connect to Device**
   - Establish BLE connection
   - Wait for connection confirmation

3. **Discover Services and Characteristics**
   - Find service: `12345678-1234-1234-1234-123456789abc`
   - Locate data characteristic: `87654321-4321-4321-4321-cba987654321`
   - Locate command characteristic: `11111111-2222-3333-4444-555555555555`

4. **Enable Notifications**
   - Subscribe to data characteristic notifications
   - Write `0x0100` to Client Characteristic Configuration Descriptor (CCCD)

5. **Start Receiving Data**
   - Data packets will stream automatically at ~100 Hz

---

## Data Streaming (Notifications)

### Data Flow Overview

```
ESP32 Device → BLE Notification → Client Application
    ↓
100 packets/sec
    ↓
Each packet: 1-10 samples
    ↓
Each sample: 8 load cells × int16
```

### Transmission Rate

- **Packet Rate:** ~100 packets per second
- **Sample Rate:** ~1000 samples per second (10 samples per packet)
- **Channels:** 8 load cells (4 local + 4 remote)
- **Update Frequency:** 1 kHz per channel

### Packet Structure

Each BLE notification contains an `OptimizedBLEPacket`:

```
┌─────────────────────────────────────────────────────┐
│ Byte 0: sample_count (uint8_t)                     │
├─────────────────────────────────────────────────────┤
│ Bytes 1-16: Sample 0 (CompactLoadCellSample)       │
│ Bytes 17-32: Sample 1 (CompactLoadCellSample)      │
│ Bytes 33-48: Sample 2 (CompactLoadCellSample)      │
│ ...                                                 │
│ Bytes 145-160: Sample 9 (CompactLoadCellSample)    │
└─────────────────────────────────────────────────────┘
Total: 1 + (sample_count × 16) bytes
Max size: 161 bytes (when sample_count = 10)
```

---

## Data Format Specification

### OptimizedBLEPacket Structure

| Field | Type | Size | Offset | Description |
|-------|------|------|--------|-------------|
| `sample_count` | `uint8_t` | 1 byte | 0 | Number of samples in this packet (1-10) |
| `samples[0..9]` | `CompactLoadCellSample[]` | 16 bytes each | 1+ | Array of load cell samples |

### CompactLoadCellSample Structure

Each sample contains 8 load cell readings (16 bytes total):

| Field | Type | Size | Offset | Description |
|-------|------|------|--------|-------------|
| `local_lc[0]` | `int16_t` | 2 bytes | 0 | Local Load Cell 1 |
| `local_lc[1]` | `int16_t` | 2 bytes | 2 | Local Load Cell 2 |
| `local_lc[2]` | `int16_t` | 2 bytes | 4 | Local Load Cell 3 |
| `local_lc[3]` | `int16_t` | 2 bytes | 6 | Local Load Cell 4 |
| `remote_lc[0]` | `int16_t` | 2 bytes | 8 | Remote Load Cell 5 |
| `remote_lc[1]` | `int16_t` | 2 bytes | 10 | Remote Load Cell 6 |
| `remote_lc[2]` | `int16_t` | 2 bytes | 12 | Remote Load Cell 7 |
| `remote_lc[3]` | `int16_t` | 2 bytes | 14 | Remote Load Cell 8 |

### Data Type Details

- **int16_t:** Signed 16-bit integer
  - Range: -32,768 to +32,767
  - Byte Order: Little-endian (LSB first)
  - Raw ADC values (not calibrated)

### Parsing Example (Pseudocode)

```python
def parse_ble_packet(data: bytes) -> dict:
    sample_count = data[0]
    samples = []
    
    for i in range(sample_count):
        offset = 1 + (i * 16)
        sample = {
            'local_lc': [
                int16_from_bytes(data[offset:offset+2]),
                int16_from_bytes(data[offset+2:offset+4]),
                int16_from_bytes(data[offset+4:offset+6]),
                int16_from_bytes(data[offset+6:offset+8]),
            ],
            'remote_lc': [
                int16_from_bytes(data[offset+8:offset+10]),
                int16_from_bytes(data[offset+10:offset+12]),
                int16_from_bytes(data[offset+12:offset+14]),
                int16_from_bytes(data[offset+14:offset+16]),
            ]
        }
        samples.append(sample)
    
    return {'sample_count': sample_count, 'samples': samples}
```

---

## Command Interface (Write)

### Sending Commands

To send a command to the device:

1. Convert command string to UTF-8 bytes
2. Write bytes to command characteristic: `11111111-2222-3333-4444-555555555555`
3. Wait for command execution (optional: monitor serial output)

### Command Format

- **Encoding:** UTF-8 string
- **Case:** Insensitive (automatically converted to uppercase)
- **Terminator:** None required (newline optional)
- **Max Length:** ~100 characters

### Command Categories

Commands are organized into 5 categories:

1. **Local Control** - Controls local Teensy only
2. **Remote Control** - Controls remote Teensy only
3. **Dual Control** - Controls both Teensys simultaneously
4. **Zero Calibration** - Tares/zeros load cells
5. **System Control** - ESP32 and status commands

---

## Command Reference

### 1. Local Teensy Control

Controls the local (wired) Teensy board.

| Command | Description | Response Time |
|---------|-------------|---------------|
| `START` | Start local data acquisition | < 1 second |
| `STOP` | Stop local data acquisition | < 1 second |
| `RESTART` | Restart local Teensy | 2-3 seconds |
| `RESET` | Reset local Teensy | 2-3 seconds |

**Example:**
```python
# Python with bleak
await client.write_gatt_char("11111111-2222-3333-4444-555555555555", b"START")
```

---

### 2. Remote Teensy Control

Controls the remote (wireless) Teensy board via radio link.

| Command | Description | Response Time |
|---------|-------------|---------------|
| `REMOTE_START` | Start remote data acquisition | < 2 seconds |
| `REMOTE_STOP` | Stop remote data acquisition | < 2 seconds |
| `REMOTE_RESTART` | Restart remote Teensy | 3-4 seconds |
| `REMOTE_RESET` | Reset remote Teensy | 3-4 seconds |

**Example:**
```python
await client.write_gatt_char("11111111-2222-3333-4444-555555555555", b"REMOTE_START")
```

---

### 3. Dual Teensy Control

Controls both local and remote Teensys simultaneously.

| Command | Description | Response Time |
|---------|-------------|---------------|
| `ALL_START` | Start both Teensys | < 2 seconds |
| `ALL_STOP` | Stop both Teensys | < 2 seconds |
| `ALL_RESTART` | Restart both Teensys | 3-4 seconds |
| `ALL_RESET` | Reset both Teensys | 3-4 seconds |

**Example:**
```python
await client.write_gatt_char("11111111-2222-3333-4444-555555555555", b"ALL_START")
```

---

### 4. Zero Calibration Commands

Tare/zero load cells to remove baseline offsets.

#### Local Calibration

| Command | Description | Response Time |
|---------|-------------|---------------|
| `ZERO` | Zero local load cells | 10-15 seconds |
| `ZERO_STATUS` | Check local zero calibration status | < 1 second |
| `ZERO_RESET` | Clear local zero offsets | < 1 second |

#### Remote Calibration

| Command | Description | Response Time |
|---------|-------------|---------------|
| `REMOTE_ZERO` | Zero remote load cells | 15-20 seconds |
| `REMOTE_ZERO_STATUS` | Check remote zero status | < 2 seconds |
| `REMOTE_ZERO_RESET` | Clear remote zero offsets | < 2 seconds |

#### Dual Calibration

| Command | Description | Response Time |
|---------|-------------|---------------|
| `ALL_ZERO` | Zero all 8 load cells | 15-20 seconds |
| `ALL_ZERO_STATUS` | Check all zero status | < 2 seconds |
| `ALL_ZERO_RESET` | Clear all zero offsets | < 2 seconds |

**Example:**
```python
# Zero all load cells
await client.write_gatt_char("11111111-2222-3333-4444-555555555555", b"ALL_ZERO")
# Wait for calibration to complete (15-20 seconds)
await asyncio.sleep(20)
```

---

### 5. System Control Commands

Control ESP32 modules and query system status.

| Command | Description | Response Time |
|---------|-------------|---------------|
| `STATUS` | Get system status (all modules) | < 1 second |
| `LOCAL_ON` | Enable local data forwarding | < 1 second |
| `LOCAL_OFF` | Disable local data forwarding | < 1 second |
| `REMOTE_ON` | Enable remote data forwarding | < 1 second |
| `REMOTE_OFF` | Disable remote data forwarding | < 1 second |

**Example:**
```python
await client.write_gatt_char("11111111-2222-3333-4444-555555555555", b"STATUS")
```

---

## Client Implementation Examples

### Python (using `bleak` library)

#### Complete Client Example

```python
import asyncio
import struct
from bleak import BleakClient, BleakScanner

# UUIDs
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_CHAR_UUID = "87654321-4321-4321-4321-cba987654321"
COMMAND_CHAR_UUID = "11111111-2222-3333-4444-555555555555"

class LoadCellBLEClient:
    def __init__(self):
        self.client = None
        self.sample_count = 0
        
    async def connect(self):
        """Scan and connect to LoadCell BLE Server"""
        print("Scanning for LoadCell_BLE_Server...")
        devices = await BleakScanner.discover(timeout=5.0)
        
        target_device = None
        for device in devices:
            if device.name == "LoadCell_BLE_Server":
                target_device = device
                break
        
        if not target_device:
            raise Exception("LoadCell_BLE_Server not found")
        
        print(f"Found device: {target_device.name} ({target_device.address})")
        self.client = BleakClient(target_device.address)
        await self.client.connect()
        print("Connected!")
        
        # Enable notifications
        await self.client.start_notify(DATA_CHAR_UUID, self.notification_handler)
        print("Notifications enabled")
    
    def notification_handler(self, sender, data: bytearray):
        """Handle incoming data notifications"""
        # Parse packet
        sample_count = data[0]
        
        print(f"\n--- Packet {self.sample_count} (contains {sample_count} samples) ---")
        
        for i in range(sample_count):
            offset = 1 + (i * 16)
            
            # Parse 8 load cells (4 local + 4 remote)
            local_lc = struct.unpack_from('<4h', data, offset)      # 4 int16 little-endian
            remote_lc = struct.unpack_from('<4h', data, offset + 8) # 4 int16 little-endian
            
            print(f"Sample {i}: Local={local_lc}, Remote={remote_lc}")
        
        self.sample_count += 1
    
    async def send_command(self, command: str):
        """Send a command to the device"""
        print(f"Sending command: {command}")
        await self.client.write_gatt_char(COMMAND_CHAR_UUID, command.encode('utf-8'))
    
    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("Disconnected")

# Usage example
async def main():
    client = LoadCellBLEClient()
    
    try:
        # Connect
        await client.connect()
        
        # Send commands
        await client.send_command("ALL_START")
        await asyncio.sleep(2)
        
        # Receive data for 10 seconds
        print("\nReceiving data for 10 seconds...")
        await asyncio.sleep(10)
        
        # Stop acquisition
        await client.send_command("ALL_STOP")
        await asyncio.sleep(1)
        
        # Disconnect
        await client.disconnect()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
```

---

### JavaScript (Node.js using `@abandonware/noble`)

```javascript
const noble = require('@abandonware/noble');

const SERVICE_UUID = '1234567812341234123412345678abc';
const DATA_CHAR_UUID = '8765432143214321432100cba987654321';
const COMMAND_CHAR_UUID = '1111111122223333444455555555555';

let dataCharacteristic = null;
let commandCharacteristic = null;

noble.on('stateChange', (state) => {
  if (state === 'poweredOn') {
    console.log('Scanning for LoadCell_BLE_Server...');
    noble.startScanning([SERVICE_UUID]);
  }
});

noble.on('discover', async (peripheral) => {
  if (peripheral.advertisement.localName === 'LoadCell_BLE_Server') {
    console.log(`Found device: ${peripheral.advertisement.localName}`);
    noble.stopScanning();
    
    await peripheral.connectAsync();
    console.log('Connected!');
    
    const {characteristics} = await peripheral.discoverSomeServicesAndCharacteristicsAsync(
      [SERVICE_UUID],
      [DATA_CHAR_UUID, COMMAND_CHAR_UUID]
    );
    
    dataCharacteristic = characteristics.find(c => c.uuid === DATA_CHAR_UUID);
    commandCharacteristic = characteristics.find(c => c.uuid === COMMAND_CHAR_UUID);
    
    // Subscribe to notifications
    dataCharacteristic.on('data', (data) => {
      parsePacket(data);
    });
    
    await dataCharacteristic.subscribeAsync();
    console.log('Notifications enabled');
    
    // Send START command
    await sendCommand('ALL_START');
  }
});

function parsePacket(data) {
  const sampleCount = data.readUInt8(0);
  
  console.log(`\n--- Packet (${sampleCount} samples) ---`);
  
  for (let i = 0; i < sampleCount; i++) {
    const offset = 1 + (i * 16);
    
    const localLc = [
      data.readInt16LE(offset),
      data.readInt16LE(offset + 2),
      data.readInt16LE(offset + 4),
      data.readInt16LE(offset + 6)
    ];
    
    const remoteLc = [
      data.readInt16LE(offset + 8),
      data.readInt16LE(offset + 10),
      data.readInt16LE(offset + 12),
      data.readInt16LE(offset + 14)
    ];
    
    console.log(`Sample ${i}: Local=[${localLc}], Remote=[${remoteLc}]`);
  }
}

async function sendCommand(command) {
  console.log(`Sending command: ${command}`);
  await commandCharacteristic.writeAsync(Buffer.from(command, 'utf-8'), false);
}
```

---

### C# (using `Windows.Devices.Bluetooth`)

```csharp
using System;
using System.Linq;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Storage.Streams;

public class LoadCellBLEClient
{
    private static readonly Guid SERVICE_UUID = Guid.Parse("12345678-1234-1234-1234-123456789abc");
    private static readonly Guid DATA_CHAR_UUID = Guid.Parse("87654321-4321-4321-4321-cba987654321");
    private static readonly Guid COMMAND_CHAR_UUID = Guid.Parse("11111111-2222-3333-4444-555555555555");
    
    private GattCharacteristic dataCharacteristic;
    private GattCharacteristic commandCharacteristic;
    
    public async Task ConnectAsync(ulong bluetoothAddress)
    {
        var device = await BluetoothLEDevice.FromBluetoothAddressAsync(bluetoothAddress);
        var gattResult = await device.GetGattServicesForUuidAsync(SERVICE_UUID);
        
        if (gattResult.Status != GattCommunicationStatus.Success)
            throw new Exception("Service not found");
        
        var service = gattResult.Services.First();
        
        // Get characteristics
        var dataCharResult = await service.GetCharacteristicsForUuidAsync(DATA_CHAR_UUID);
        var commandCharResult = await service.GetCharacteristicsForUuidAsync(COMMAND_CHAR_UUID);
        
        dataCharacteristic = dataCharResult.Characteristics.First();
        commandCharacteristic = commandCharResult.Characteristics.First();
        
        // Enable notifications
        await dataCharacteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
            GattClientCharacteristicConfigurationDescriptorValue.Notify);
        
        dataCharacteristic.ValueChanged += DataCharacteristic_ValueChanged;
        
        Console.WriteLine("Connected and notifications enabled");
    }
    
    private void DataCharacteristic_ValueChanged(GattCharacteristic sender, 
                                                  GattValueChangedEventArgs args)
    {
        var reader = DataReader.FromBuffer(args.CharacteristicValue);
        byte sampleCount = reader.ReadByte();
        
        Console.WriteLine($"\n--- Packet ({sampleCount} samples) ---");
        
        for (int i = 0; i < sampleCount; i++)
        {
            short[] localLc = new short[4];
            short[] remoteLc = new short[4];
            
            for (int j = 0; j < 4; j++)
                localLc[j] = reader.ReadInt16();
            
            for (int j = 0; j < 4; j++)
                remoteLc[j] = reader.ReadInt16();
            
            Console.WriteLine($"Sample {i}: Local=[{string.Join(", ", localLc)}], " +
                            $"Remote=[{string.Join(", ", remoteLc)}]");
        }
    }
    
    public async Task SendCommandAsync(string command)
    {
        var writer = new DataWriter();
        writer.WriteString(command);
        
        await commandCharacteristic.WriteValueAsync(writer.DetachBuffer());
        Console.WriteLine($"Sent command: {command}");
    }
}
```

---

## Troubleshooting

### Connection Issues

| Problem | Solution |
|---------|----------|
| Device not found during scan | Ensure ESP32 is powered on and BLE advertising is active |
| Connection fails | Check if another client is already connected (only 1 concurrent connection supported) |
| Notifications not received | Verify CCCD descriptor is set to `0x0100` |

### Data Reception Issues

| Problem | Solution |
|---------|----------|
| Packet loss / gaps | BLE connection quality issue - reduce distance or interference |
| Incorrect values | Check byte order (little-endian) and data type (int16) |
| No data flowing | Ensure Teensy boards are started (`ALL_START` command) |

### Command Issues

| Problem | Solution |
|---------|----------|
| Command not executing | Check command spelling (case-insensitive but must match exactly) |
| Timeout waiting for response | Commands forwarded to Teensy may take 1-20 seconds depending on type |
| ZERO command takes too long | Normal - calibration takes 10-20 seconds to average samples |

### Performance Optimization

| Issue | Recommendation |
|-------|---------------|
| High CPU usage parsing packets | Batch process multiple packets before updating UI |
| Memory growth | Implement circular buffer for sample storage |
| Dropped notifications | Windows BLE stack may drop packets at high rates - consider aggregation |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     BLE Client (Your App)                   │
│  ┌──────────────────────┐      ┌──────────────────────┐   │
│  │  Notification Handler │      │  Command Sender     │   │
│  │  (Receives Data)      │      │  (Sends Commands)   │   │
│  └──────────┬───────────┘      └──────────▲───────────┘   │
└─────────────┼──────────────────────────────┼──────────────┘
              │ BLE Notify                    │ BLE Write
              │ 100 pps                       │ On-demand
┌─────────────▼──────────────────────────────┴──────────────┐
│              ESP32 BLE Slave (This Device)                 │
│  ┌──────────────────────────────────────────────────────┐ │
│  │  Service: 12345678-1234-1234-1234-123456789abc      │ │
│  │  ┌────────────────────┐  ┌────────────────────────┐ │ │
│  │  │ Data Characteristic │  │ Command Characteristic │ │ │
│  │  │ (Notify)            │  │ (Write)                │ │ │
│  │  └─────────▲───────────┘  └─────────┬─────────────┘ │ │
│  └────────────┼──────────────────────────┼──────────────┘ │
│               │ 1000 Hz Timer            │ UART Fwd       │
│  ┌────────────┴──────────────┐  ┌───────▼──────────────┐ │
│  │  Redis-like Data Store    │  │  Command Forwarder   │ │
│  │  (Local + Remote Queues)  │  │  (to ESP32 RX Radio) │ │
│  └────────────▲───────────────┘  └──────────────────────┘ │
└───────────────┼──────────────────────────────────────────┘
                │ UART @ 921600 bps
┌───────────────▼──────────────────────────────────────────┐
│              ESP32 RX Radio (Master)                       │
│  ┌──────────────────┐        ┌──────────────────┐        │
│  │ Local Teensy     │        │ Remote Radio RX  │        │
│  │ (UART)           │        │ (Wireless)       │        │
│  └──────────────────┘        └──────────────────┘        │
└──────────────────────────────────────────────────────────┘
```

---

## Data Rate Calculations

### Expected Throughput

- **Sample Rate:** 1000 samples/second (1 kHz)
- **Packet Rate:** 100 packets/second
- **Samples per Packet:** 10 samples (average)
- **Packet Size:** 161 bytes (1 + 10×16)
- **Data Rate:** 161 bytes × 100 pps = **16,100 bytes/sec** (~128.8 kbps)

### BLE Bandwidth Usage

- **BLE 4.2 Maximum:** ~1 Mbps (theoretical)
- **Practical Limit:** ~100-200 kbps
- **This System:** ~129 kbps (**within safe limits**)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025 | Initial release with optimized int16 format |

---

## Support & Contact

For issues or questions:
- Check serial monitor output for debugging (921600 baud)
- Use `STATS` serial command for system diagnostics
- Use `HELP` serial command for available commands

---

## License

© 2025 - ESP32 Load Cell BLE Server API Documentation


