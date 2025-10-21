# ESP32 Load Cell BLE Server - Quick Reference

## Connection Info

**Device Name:** `LoadCell_BLE_Server`

| UUID Type | UUID | Properties |
|-----------|------|------------|
| Service | `12345678-1234-1234-1234-123456789abc` | - |
| Data Characteristic | `87654321-4321-4321-4321-cba987654321` | Read, Notify |
| Command Characteristic | `11111111-2222-3333-4444-555555555555` | Write, Write No Response |

---

## Quick Connect (Python)

```python
import asyncio
import struct
from bleak import BleakClient

DATA_UUID = "87654321-4321-4321-4321-cba987654321"
CMD_UUID = "11111111-2222-3333-4444-555555555555"

async def main():
    # Connect
    async with BleakClient("LoadCell_BLE_Server") as client:
        # Handle data
        def callback(sender, data):
            count = data[0]
            for i in range(count):
                offset = 1 + i*16
                local = struct.unpack_from('<4h', data, offset)
                remote = struct.unpack_from('<4h', data, offset+8)
                print(f"Local: {local}, Remote: {remote}")
        
        await client.start_notify(DATA_UUID, callback)
        
        # Send command
        await client.write_gatt_char(CMD_UUID, b"ALL_START")
        
        # Receive for 10 seconds
        await asyncio.sleep(10)
        
        await client.write_gatt_char(CMD_UUID, b"ALL_STOP")

asyncio.run(main())
```

---

## Data Format Cheat Sheet

### BLE Packet Structure (Max 161 bytes)

```
Byte 0:     sample_count (1-10)
Bytes 1-16:   Sample 0 (8 × int16_t)
Bytes 17-32:  Sample 1 (8 × int16_t)
...
Bytes 145-160: Sample 9 (8 × int16_t)
```

### Sample Structure (16 bytes)

```
Offset | Field          | Type    | Bytes
-------|----------------|---------|-------
0-1    | local_lc[0]    | int16_t | 2
2-3    | local_lc[1]    | int16_t | 2
4-5    | local_lc[2]    | int16_t | 2
6-7    | local_lc[3]    | int16_t | 2
8-9    | remote_lc[0]   | int16_t | 2
10-11  | remote_lc[1]   | int16_t | 2
12-13  | remote_lc[2]   | int16_t | 2
14-15  | remote_lc[3]   | int16_t | 2
```

**Note:** All values are little-endian signed 16-bit integers (-32,768 to +32,767)

---

## Parse Functions

### Python
```python
def parse_packet(data):
    sample_count = data[0]
    samples = []
    for i in range(sample_count):
        offset = 1 + i * 16
        local = struct.unpack_from('<4h', data, offset)
        remote = struct.unpack_from('<4h', data, offset + 8)
        samples.append({'local': local, 'remote': remote})
    return samples
```

### JavaScript
```javascript
function parsePacket(data) {
  const view = new DataView(data.buffer);
  const sampleCount = view.getUint8(0);
  const samples = [];
  
  for (let i = 0; i < sampleCount; i++) {
    const offset = 1 + i * 16;
    const local = [0,2,4,6].map(o => view.getInt16(offset + o, true));
    const remote = [8,10,12,14].map(o => view.getInt16(offset + o, true));
    samples.push({local, remote});
  }
  return samples;
}
```

### C#
```csharp
public static List<Sample> ParsePacket(byte[] data)
{
    int sampleCount = data[0];
    var samples = new List<Sample>();
    
    for (int i = 0; i < sampleCount; i++)
    {
        int offset = 1 + i * 16;
        var sample = new Sample
        {
            Local = new short[4],
            Remote = new short[4]
        };
        
        for (int j = 0; j < 4; j++)
        {
            sample.Local[j] = BitConverter.ToInt16(data, offset + j*2);
            sample.Remote[j] = BitConverter.ToInt16(data, offset + 8 + j*2);
        }
        samples.Add(sample);
    }
    return samples;
}
```

---

## Essential Commands

### Basic Control
```
ALL_START          # Start both Teensys (most common)
ALL_STOP           # Stop both Teensys
START              # Start local only
REMOTE_START       # Start remote only
```

### Calibration
```
ALL_ZERO           # Zero all 8 load cells (takes 15-20s)
ZERO               # Zero local 4 load cells only
REMOTE_ZERO        # Zero remote 4 load cells only
ALL_ZERO_STATUS    # Check calibration status
```

### System
```
STATUS             # Get system status
LOCAL_ON           # Enable local data forwarding
REMOTE_ON          # Enable remote data forwarding
```

---

## Performance Specs

| Metric | Value |
|--------|-------|
| Sample Rate | 1000 samples/sec (1 kHz) |
| Packet Rate | 100 packets/sec |
| Samples per Packet | 10 (average) |
| Packet Size | 161 bytes (typical) |
| Data Rate | ~16.1 KB/sec (~129 kbps) |
| Channels | 8 (4 local + 4 remote) |
| Data Type | int16_t (-32,768 to +32,767) |
| Latency | 15-30 ms (typical) |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Can't find device | Check ESP32 powered on, BLE enabled on client |
| No data received | Enable notifications, send `ALL_START` command |
| Connection drops | Check distance/interference, only 1 client at a time |
| Parsing errors | Verify little-endian int16_t, check sample_count |
| Command not working | Commands are case-insensitive, check spelling |

---

## Common Workflows

### 1. Basic Data Collection
```python
1. Connect to device
2. Enable notifications on DATA_UUID
3. Send "ALL_START"
4. Receive data (100 packets/sec)
5. Send "ALL_STOP"
6. Disconnect
```

### 2. Zero Calibration
```python
1. Connect to device
2. Send "ALL_ZERO"
3. Wait 20 seconds
4. Send "ALL_ZERO_STATUS" to verify
5. Send "ALL_START" to begin measurements
```

### 3. Continuous Monitoring
```python
1. Connect to device
2. Enable notifications
3. Send "ALL_START"
4. Process incoming data continuously
5. Handle reconnection on disconnect
```

---

## Code Templates

### Complete Python Client (Minimal)
```python
import asyncio
from bleak import BleakClient
import struct

async def main():
    async with BleakClient("LoadCell_BLE_Server") as client:
        def data_handler(sender, data):
            count = data[0]
            for i in range(count):
                offs = 1 + i*16
                local = struct.unpack_from('<4h', data, offs)
                remote = struct.unpack_from('<4h', data, offs+8)
                print(local, remote)
        
        await client.start_notify(
            "87654321-4321-4321-4321-cba987654321", 
            data_handler
        )
        await client.write_gatt_char(
            "11111111-2222-3333-4444-555555555555", 
            b"ALL_START"
        )
        await asyncio.sleep(10)

asyncio.run(main())
```

### Complete JavaScript Client (Minimal)
```javascript
const noble = require('@abandonware/noble');

noble.on('discover', async (peripheral) => {
  if (peripheral.advertisement.localName === 'LoadCell_BLE_Server') {
    await peripheral.connectAsync();
    const {characteristics} = await peripheral
      .discoverSomeServicesAndCharacteristicsAsync(
        ['12345678123412341234123456789abc'],
        ['87654321432143214321cba987654321', '11111111222233334444555555555555']
      );
    
    const dataChar = characteristics[0];
    const cmdChar = characteristics[1];
    
    dataChar.on('data', (data) => {
      // Parse data here
      console.log('Received:', data);
    });
    
    await dataChar.subscribeAsync();
    await cmdChar.writeAsync(Buffer.from('ALL_START'), false);
  }
});

noble.startScanning();
```

---

## Testing Commands (Serial Monitor)

Connect to ESP32 serial port at 921600 baud:

```
HELP           # Show all commands
STATS          # Show statistics
REDIS_STATUS   # Show queue status
BATCH_STATUS   # Show BLE batch status
RX_STATUS      # Get ESP32 RX Radio status
UART_STATUS    # Show UART status
DEBUG_QUEUE    # Show next samples in queue
RESET_STATS    # Clear statistics
```

---

## Data Flow Summary

```
Teensy (Local) ──UART──┐
                        ├─► ESP32 RX Radio ──UART──► ESP32 BLE Slave ──BLE──► Your Client
Teensy (Remote) ─Radio─┘
                         (400 pps)           (1000 Hz)        (100 pps)
                         (24 bytes)          (Redis)          (161 bytes)
```

---

## Support

For detailed information, see:
- `BLE_API_DOCUMENTATION.md` - Complete API reference
- `BLE_DATA_FLOW_DIAGRAM.md` - Detailed system architecture
- Serial monitor output (921600 baud) - Real-time diagnostics

---

**Quick Reference Version:** 1.0  
**Last Updated:** October 2025








