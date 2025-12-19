# ESP32 BLE Load Cell Server - Client Integration Guide

This document describes how to connect to the ESP32 BLE Load Cell Server, subscribe to sensor data, send commands, and receive JSON responses.

---

## BLE Connection Details

### Device Information

| Property | Value |
|----------|-------|
| **Device Name** | `LoadCell_BLE_Server` |
| **Service UUID** | `12345678-1234-1234-1234-123456789abc` |

### Characteristics

| Characteristic | UUID | Properties | Purpose |
|----------------|------|------------|---------|
| **Data** | `87654321-4321-4321-4321-cba987654321` | NOTIFY | Binary sensor data stream |
| **Cmd** | `11111111-2222-3333-4444-555555555555` | WRITE, NOTIFY | Send commands, receive JSON responses |

---

## Quick Start

### 1. Connect
Scan for `LoadCell_BLE_Server` and connect.

### 2. Subscribe to Cmd Characteristic
Enable notifications on `11111111-2222-3333-4444-555555555555` to receive command responses.

### 3. Send Commands
Write text to the same Cmd characteristic (e.g., `LOCAL_PING`).

### 4. Receive JSON Response
Response arrives as notification: `{"target":"LOCAL","cmd":"PING","ok":true,"ms":45}`

### 5. Subscribe to Data (Optional)
Enable notifications on `87654321-4321-4321-4321-cba987654321` for sensor data stream.

---

## JSON Response Format

All command responses are JSON objects with these fields:

| Field | Type | Description |
|-------|------|-------------|
| `target` | string | `"LOCAL"`, `"REMOTE"`, `"ALL"`, or `"BLE"` |
| `cmd` | string | Command name (e.g., `"PING"`, `"TARE"`, `"SHOW"`) |
| `ok` | boolean | Success status |
| `ms` | number | Round-trip time in milliseconds |
| `err` | string | Error message (only if `ok` is false) |

### Response Examples

```json
// PING success
{"target":"LOCAL","cmd":"PING","ok":true,"ms":45}

// TARE success
{"target":"LOCAL","cmd":"TARE","ok":true,"ms":650}

// TARE failure (timeout)
{"target":"LOCAL","cmd":"TARE","ok":false,"err":"TIMEOUT","ms":5001}

// SHOW - calibration status
{"target":"LOCAL","cmd":"SHOW","lc":[
  {"off":38853,"a":0.015743,"n":2},
  {"off":12345,"a":0.014892,"n":2},
  {"off":-1234,"a":0.016021,"n":2},
  {"off":-5678,"a":0.015156,"n":2}
],"ms":52}

// READ - calibrated values (10g units)
{"target":"LOCAL","cmd":"READ","v":[100,150,120,130,500],"ms":35}

// PTS - point counts per channel
{"target":"LOCAL","cmd":"PTS","n":[2,2,2,2],"ms":30}

// ADD calibration point
{"target":"LOCAL","cmd":"ADD","ok":true,"ms":850}

// Control command
{"target":"LOCAL","cmd":"START","ok":true,"ms":120}

// BAT - battery status
{"target":"BLE","cmd":"BAT","ok":true,"local":{"v":4.12,"pct":85.0},"remote":{"v":3.98,"pct":72.0},"ms":0}
```

---

## Sensor Data Format (Binary)

The Data characteristic sends batched load cell samples at ~100 packets/second (1000 samples/second).

### Packet Structure

```
Byte 0:        sample_count (1-10)
Bytes 1-160:   samples[sample_count]
```

**Maximum packet size: 161 bytes**

### Each Sample (16 bytes)

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| 0-1 | 2 bytes | int16 LE | Local Load Cell 1 |
| 2-3 | 2 bytes | int16 LE | Local Load Cell 2 |
| 4-5 | 2 bytes | int16 LE | Local Load Cell 3 |
| 6-7 | 2 bytes | int16 LE | Local Load Cell 4 |
| 8-9 | 2 bytes | int16 LE | Remote Load Cell 5 |
| 10-11 | 2 bytes | int16 LE | Remote Load Cell 6 |
| 12-13 | 2 bytes | int16 LE | Remote Load Cell 7 |
| 14-15 | 2 bytes | int16 LE | Remote Load Cell 8 |

**Value Range:** -32768 to +32767 (int16)

---

## Available Commands

### Control Commands

| Command | Description |
|---------|-------------|
| `START` | Start local data acquisition |
| `STOP` | Stop local data acquisition |
| `RESTART` | Restart local Teensy |
| `RESET` | Reset local Teensy |
| `REMOTE_START` | Start remote data acquisition |
| `REMOTE_STOP` | Stop remote data acquisition |
| `REMOTE_RESTART` | Restart remote Teensy |
| `REMOTE_RESET` | Reset remote Teensy |
| `ALL_START` | Start both |
| `ALL_STOP` | Stop both |

### Ping Commands

| Command | Response |
|---------|----------|
| `LOCAL_PING` | `{"target":"LOCAL","cmd":"PING","ok":true,"ms":XX}` |
| `REMOTE_PING` | `{"target":"REMOTE","cmd":"PING","ok":true,"ms":XX}` |

### Calibration Commands

| Command | Description |
|---------|-------------|
| `LOCAL_CAL_TARE` | Zero load cells (no weight) |
| `LOCAL_CAL_SHOW` | Show calibration status |
| `LOCAL_CAL_READ` | Read calibrated values (10g units) |
| `LOCAL_CAL_ADD_<kg>` | Add calibration point (e.g., `LOCAL_CAL_ADD_10`) |
| `LOCAL_CAL_ADD_CH_<ch>_<kg>` | Add point to specific channel |
| `LOCAL_CAL_CLEAR` | Clear all calibration |
| `LOCAL_CAL_POINTS` | Show point counts |
| `REMOTE_CAL_*` | Same commands for remote Teensy |

### Statistics Commands

| Command | Description |
|---------|-------------|
| `STATS` | Show BLE Slave statistics |
| `RESET_STATS` | Reset statistics counters |

### Battery Command

| Command | Description |
|---------|-------------|
| `BAT` | Get battery status for both local and remote batteries |

**BAT Response Format:**
```json
{
  "target": "BLE",
  "cmd": "BAT",
  "ok": true,
  "local": {"v": 4.12, "pct": 85.0},
  "remote": {"v": 3.98, "pct": 72.0},
  "ms": 0
}
```

| Field | Description |
|-------|-------------|
| `local.v` | Local battery voltage (V) |
| `local.pct` | Local battery percentage (0-100%) |
| `remote.v` | Remote battery voltage (V) |
| `remote.pct` | Remote battery percentage (0-100%) |

---

## Parsing Examples

### Python (with bleak)

```python
import struct
import json
from bleak import BleakClient

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_UUID = "87654321-4321-4321-4321-cba987654321"
CMD_UUID = "11111111-2222-3333-4444-555555555555"

def parse_sensor_data(data: bytes) -> list:
    """Parse binary sensor data packet."""
    sample_count = data[0]
    samples = []
    for i in range(sample_count):
        offset = 1 + (i * 16)
        values = struct.unpack_from('<8h', data, offset)
        samples.append({
            'local': list(values[0:4]),
            'remote': list(values[4:8])
        })
    return samples

def parse_json_response(data: bytes) -> dict:
    """Parse JSON command response."""
    return json.loads(data.decode('utf-8'))

async def main():
    async with BleakClient("XX:XX:XX:XX:XX:XX") as client:
        # Handler for command responses (JSON)
        def cmd_handler(sender, data):
            response = parse_json_response(data)
            print(f"Response: {response}")
        
        # Handler for sensor data (binary)
        def data_handler(sender, data):
            samples = parse_sensor_data(data)
            for s in samples:
                print(f"L:{s['local']} R:{s['remote']}")
        
        # Subscribe to responses
        await client.start_notify(CMD_UUID, cmd_handler)
        
        # Send command
        await client.write_gatt_char(CMD_UUID, b"LOCAL_PING")
        
        # Subscribe to sensor data
        await client.start_notify(DATA_UUID, data_handler)
        await client.write_gatt_char(CMD_UUID, b"ALL_START")
        
        # Keep running...
        await asyncio.sleep(60)
```

### JavaScript (Web Bluetooth)

```javascript
const SERVICE_UUID = '12345678-1234-1234-1234-123456789abc';
const DATA_UUID = '87654321-4321-4321-4321-cba987654321';
const CMD_UUID = '11111111-2222-3333-4444-555555555555';

function parseSensorData(dataView) {
    const sampleCount = dataView.getUint8(0);
    const samples = [];
    for (let i = 0; i < sampleCount; i++) {
        const offset = 1 + (i * 16);
        samples.push({
            local: [
                dataView.getInt16(offset + 0, true),
                dataView.getInt16(offset + 2, true),
                dataView.getInt16(offset + 4, true),
                dataView.getInt16(offset + 6, true),
            ],
            remote: [
                dataView.getInt16(offset + 8, true),
                dataView.getInt16(offset + 10, true),
                dataView.getInt16(offset + 12, true),
                dataView.getInt16(offset + 14, true),
            ]
        });
    }
    return samples;
}

async function connect() {
    const device = await navigator.bluetooth.requestDevice({
        filters: [{ name: 'LoadCell_BLE_Server' }],
        optionalServices: [SERVICE_UUID]
    });
    
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(SERVICE_UUID);
    
    // Get characteristics
    const cmdChar = await service.getCharacteristic(CMD_UUID);
    const dataChar = await service.getCharacteristic(DATA_UUID);
    
    // Subscribe to command responses (JSON)
    await cmdChar.startNotifications();
    cmdChar.addEventListener('characteristicvaluechanged', (event) => {
        const decoder = new TextDecoder();
        const json = JSON.parse(decoder.decode(event.target.value));
        console.log('Response:', json);
    });
    
    // Subscribe to sensor data (binary)
    await dataChar.startNotifications();
    dataChar.addEventListener('characteristicvaluechanged', (event) => {
        const samples = parseSensorData(event.target.value);
        console.log('Samples:', samples);
    });
    
    // Send command
    const encoder = new TextEncoder();
    await cmdChar.writeValue(encoder.encode('LOCAL_PING'));
    await cmdChar.writeValue(encoder.encode('ALL_START'));
}
```

### Flutter (Dart)

```dart
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

const serviceUuid = Guid('12345678-1234-1234-1234-123456789abc');
const dataUuid = Guid('87654321-4321-4321-4321-cba987654321');
const cmdUuid = Guid('11111111-2222-3333-4444-555555555555');

List<Map<String, List<int>>> parseSensorData(Uint8List data) {
  final sampleCount = data[0];
  final samples = <Map<String, List<int>>>[];
  
  for (var i = 0; i < sampleCount; i++) {
    final offset = 1 + (i * 16);
    final byteData = ByteData.sublistView(data, offset, offset + 16);
    
    samples.add({
      'local': [
        byteData.getInt16(0, Endian.little),
        byteData.getInt16(2, Endian.little),
        byteData.getInt16(4, Endian.little),
        byteData.getInt16(6, Endian.little),
      ],
      'remote': [
        byteData.getInt16(8, Endian.little),
        byteData.getInt16(10, Endian.little),
        byteData.getInt16(12, Endian.little),
        byteData.getInt16(14, Endian.little),
      ],
    });
  }
  return samples;
}

Map<String, dynamic> parseJsonResponse(Uint8List data) {
  return jsonDecode(utf8.decode(data));
}

Future<void> connectAndStream(BluetoothDevice device) async {
  await device.connect();
  final services = await device.discoverServices();
  final service = services.firstWhere((s) => s.uuid == serviceUuid);
  
  final cmdChar = service.characteristics.firstWhere((c) => c.uuid == cmdUuid);
  final dataChar = service.characteristics.firstWhere((c) => c.uuid == dataUuid);
  
  // Subscribe to command responses
  await cmdChar.setNotifyValue(true);
  cmdChar.value.listen((data) {
    final response = parseJsonResponse(Uint8List.fromList(data));
    print('Response: $response');
  });
  
  // Subscribe to sensor data
  await dataChar.setNotifyValue(true);
  dataChar.value.listen((data) {
    final samples = parseSensorData(Uint8List.fromList(data));
    print('Samples: $samples');
  });
  
  // Send commands
  await cmdChar.write(utf8.encode('LOCAL_PING'));
  await cmdChar.write(utf8.encode('ALL_START'));
}
```

---

## Calibration Workflow

### Step 1: Clear Previous Calibration
```
Send: LOCAL_CAL_CLEAR
Response: {"target":"LOCAL","cmd":"CLEAR","ok":true,"ms":XX}
```

### Step 2: Tare (Zero with No Load)
Remove all weight from the platform.
```
Send: LOCAL_CAL_TARE
Response: {"target":"LOCAL","cmd":"TARE","ok":true,"ms":650}
```

### Step 3: Add Known Weights
Place known weight (e.g., 10 kg) on platform.
```
Send: LOCAL_CAL_ADD_10
Response: {"target":"LOCAL","cmd":"ADD","ok":true,"ms":850}
```

Repeat with more weights for better accuracy:
```
Send: LOCAL_CAL_ADD_20
Send: LOCAL_CAL_ADD_50
```

### Step 4: Verify Calibration
```
Send: LOCAL_CAL_SHOW
Response: {"target":"LOCAL","cmd":"SHOW","lc":[
  {"off":38853,"a":0.015743,"n":3},
  ...
],"ms":52}
```

The `n` field shows number of calibration points per channel.

### Step 5: Read Calibrated Values
```
Send: LOCAL_CAL_READ
Response: {"target":"LOCAL","cmd":"READ","v":[100,150,120,130,500],"ms":35}
```

Values are in 10g units (divide by 100 for kg).

---

## Quick Reference

### UUIDs
```
Service:   12345678-1234-1234-1234-123456789abc
Data:      87654321-4321-4321-4321-cba987654321  (NOTIFY)
Cmd:       11111111-2222-3333-4444-555555555555  (WRITE + NOTIFY)

Standard Battery Service: 0x180F
Battery Level Char:       0x2A19  (READ + NOTIFY, shows MIN of both batteries)
```

### Data Specs
| Property | Value |
|----------|-------|
| Channels | 8 (4 local + 4 remote) |
| Sample Rate | 1000 Hz |
| Packet Rate | ~100/sec |
| Samples/Packet | 1-10 |
| Bytes/Sample | 16 |
| Max Packet | 161 bytes |
| Value Range | int16 (-32768 to +32767) |

### Command Timeouts
| Type | Timeout |
|------|---------|
| Normal | 5 seconds |
| Calibration | 15 seconds |

---

## Battery Monitoring

The system has two batteries (local and remote). There are two ways to monitor battery status:

### 1. BAT Command (Detailed)

Send `BAT` to get full battery information:

```
Send: BAT
Response: {"target":"BLE","cmd":"BAT","ok":true,"local":{"v":4.12,"pct":85.0},"remote":{"v":3.98,"pct":72.0},"ms":0}
```

### 2. Standard BLE Battery Service (Quick)

The device also exposes a standard BLE Battery Service (0x180F) that any BLE diagnostic tool can read automatically:

- **Service UUID:** `0x180F`
- **Battery Level Characteristic:** `0x2A19`
- **Value:** Single byte 0-100%, representing the **minimum** of both batteries

This means if local is 85% and remote is 72%, the standard battery level will show 72% (worst-case).

---

## Troubleshooting

### No Sensor Data
1. Enable notifications on Data characteristic
2. Send `ALL_START` command
3. Check hardware connections

### No Command Response
1. Enable notifications on Cmd characteristic (same one you write to)
2. Check command spelling (case-insensitive)
3. Wait for timeout if device is unresponsive

### Connection Issues
- Server supports only 1 client at a time
- Server auto-restarts advertising after disconnect
- Reconnect and re-subscribe to notifications

