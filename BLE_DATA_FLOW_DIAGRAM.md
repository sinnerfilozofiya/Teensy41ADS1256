# ESP32 Load Cell BLE - Data Flow Diagram

## System Overview

This document provides detailed data flow diagrams for the ESP32 BLE Load Cell Server system.

---

## High-Level System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Physical Hardware                            │
├─────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌──────────────┐            ┌──────────────┐                       │
│  │ Teensy 4.1   │◄──────────►│ Teensy 4.1   │                       │
│  │ (Local)      │  Wireless  │ (Remote)     │                       │
│  │              │   Radio    │              │                       │
│  │ 4× ADS1256   │            │ 4× ADS1256   │                       │
│  │ Load Cells   │            │ Load Cells   │                       │
│  │ 1-4          │            │ 5-8          │                       │
│  └──────┬───────┘            └──────┬───────┘                       │
│         │ UART                      │ Radio                          │
│         │ 921600                    │ 2.4GHz                         │
│         ▼                           ▼                                │
│  ┌──────────────────────────────────────────┐                       │
│  │     ESP32 RX Radio (Master)              │                       │
│  │  - Receives local data via UART          │                       │
│  │  - Receives remote data via radio        │                       │
│  │  - Forwards commands to both Teensys     │                       │
│  └──────────────┬───────────────────────────┘                       │
│                 │ UART @ 921600 bps                                 │
│                 │ 24-byte packets                                   │
│                 ▼                                                    │
│  ┌──────────────────────────────────────────┐                       │
│  │     ESP32 BLE Slave (This Device)        │                       │
│  │  - Buffers data in Redis-like queues     │                       │
│  │  - Combines local + remote at 1000 Hz    │                       │
│  │  - Streams via BLE notifications          │                       │
│  │  - Receives commands via BLE write        │                       │
│  └──────────────┬───────────────────────────┘                       │
│                 │ BLE 4.2                                            │
│                 │ ~129 kbps                                          │
│                 ▼                                                    │
│  ┌──────────────────────────────────────────┐                       │
│  │     Your BLE Client Application          │                       │
│  │  - Python / JavaScript / C# / etc.       │                       │
│  │  - Receives 8-channel data               │                       │
│  │  - Sends control commands                │                       │
│  └──────────────────────────────────────────┘                       │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow: UART → Redis Store → BLE Client

### Step 1: UART Reception (ESP32 RX Radio → BLE Slave)

```
ESP32 RX Radio                         ESP32 BLE Slave
─────────────────                      ──────────────────
     │
     │ UART TX (Pin 17)
     │ 921600 bps
     │ 24-byte packets
     │ ~400 pps
     ▼
     │◄──────────────────────────────►│ UART RX (Pin 18)
                                        │
                                        ▼
                                  ┌──────────────┐
                                  │ UART RX Task │ (Core 0, Priority 23)
                                  │  - Sync scan │
                                  │  - CRC check │
                                  │  - Type parse│
                                  └──────┬───────┘
                                         │
                                         ▼
                        ┌────────────────┴─────────────────┐
                        │                                   │
              Type = 'L' (Local)              Type = 'R' (Remote)
                        │                                   │
                        ▼                                   ▼
          ┌─────────────────────────┐      ┌─────────────────────────┐
          │ add_to_redis_queue('L') │      │ add_to_redis_queue('R') │
          └─────────────────────────┘      └─────────────────────────┘
```

#### UART Packet Format (24 bytes)

```
 Byte Offset │ Field          │ Type      │ Size    │ Description
─────────────┼────────────────┼───────────┼─────────┼──────────────────────
 0-1         │ sync           │ uint8[2]  │ 2 bytes │ 0xAA, 0x55
 2           │ type           │ char      │ 1 byte  │ 'L' / 'R' / 'C'
 3-4         │ frame_idx      │ uint16_t  │ 2 bytes │ Frame index
 5           │ sample_idx     │ uint8_t   │ 1 byte  │ Sample index (0-9)
 6-21        │ lc[4]          │ int32_t[4]│ 16 bytes│ 4 load cell values
 22-23       │ crc16          │ uint16_t  │ 2 bytes │ CRC-16-CCITT-FALSE
─────────────┴────────────────┴───────────┴─────────┴──────────────────────
Total: 24 bytes per packet
```

---

### Step 2: Redis-like Data Store

```
┌─────────────────────────────────────────────────────────────┐
│                     Redis Store Structure                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────────────────┐  ┌──────────────────────────┐│
│  │   Local Queue (20 slots) │  │  Remote Queue (20 slots) ││
│  ├──────────────────────────┤  ├──────────────────────────┤│
│  │ [0] ┌────────────────┐   │  │ [0] ┌────────────────┐   ││
│  │     │ lc[4]          │   │  │     │ lc[4]          │   ││
│  │     │ timestamp      │   │  │     │ timestamp      │   ││
│  │     │ frame_idx      │   │  │     │ frame_idx      │   ││
│  │     │ sample_idx     │   │  │     │ sample_idx     │   ││
│  │     │ consumed: bool │   │  │     │ consumed: bool │   ││
│  │     └────────────────┘   │  │     └────────────────┘   ││
│  │ [1] ...                  │  │ [1] ...                  ││
│  │ [2] ...                  │  │ [2] ...                  ││
│  │ ...                      │  │ ...                      ││
│  │ [19]                     │  │ [19]                     ││
│  └──────────────────────────┘  └──────────────────────────┘│
│         ▲                              ▲                     │
│         │ head/tail pointers           │ head/tail pointers │
│         │ local_count                  │ remote_count       │
│                                                               │
│  ┌──────────────────────────┐  ┌──────────────────────────┐│
│  │ Last Local LC (Fallback) │  │ Last Remote LC (Fallback)││
│  │ last_local_lc[4]         │  │ last_remote_lc[4]        ││
│  │ has_local_data: bool     │  │ has_remote_data: bool    ││
│  └──────────────────────────┘  └──────────────────────────┘│
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

#### Queue Operations

**Add to Queue:**
```
1. UART packet arrives with type 'L' or 'R'
2. Check if queue has space (count < 20)
3. If space:
   - Add to tail position
   - Increment tail pointer (circular)
   - Increment count
4. If full:
   - Overwrite tail position
   - Increment both tail and head (circular)
5. Mark sample as unconsumed
6. Update last_known value
```

**Get from Queue:**
```
1. Timer interrupt (1000 Hz) requests sample
2. Check local queue for unconsumed sample
3. If found:
   - Copy data
   - Mark as consumed
   - Remove from queue (increment head)
4. If not found:
   - Use last_known fallback
5. Repeat for remote queue
6. Combine both → 8-channel sample
```

---

### Step 3: Timer-Based Sampling (1000 Hz)

```
Hardware Timer (1 MHz base frequency)
         │
         │ Triggers every 1000 ticks (1 ms)
         │ = 1000 Hz
         ▼
  ┌────────────────────┐
  │ timer_sample_callback()  │ (IRAM ISR)
  │  - Increment counter     │
  │  - Set flag: timer_sample_ready = true │
  └────────┬───────────┘
           │ Flag
           ▼
  ┌────────────────────┐
  │ timer_processing_task() │ (Core 1, Priority 24)
  │  - Polls flag every 1ms  │
  │  - Calls create_8channel_sample_from_redis() │
  └────────┬───────────┘
           │
           ▼
  ┌─────────────────────────────────────┐
  │ create_8channel_sample_from_redis() │
  ├─────────────────────────────────────┤
  │ 1. get_next_redis_sample()          │
  │    → local_lc[4], remote_lc[4]      │
  │                                      │
  │ 2. Convert int32 → int16 (clamp)    │
  │    → CompactLoadCellSample          │
  │                                      │
  │ 3. Add to current_ble_packet        │
  │    → samples[current_sample_count]  │
  │                                      │
  │ 4. Increment current_sample_count   │
  │                                      │
  │ 5. Check if batch ready:            │
  │    - Full (10 samples) OR           │
  │    - Rate limit (10 samples = 10ms) │
  │                                      │
  │ 6. If ready: send_current_batch()   │
  └─────────────────────────────────────┘
```

**Timing Diagram:**
```
Timer ISR:     ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲    ▲
               │    │    │    │    │    │    │    │    │    │    │
Time (ms):     0    1    2    3    4    5    6    7    8    9    10
               │    │    │    │    │    │    │    │    │    │    │
Samples:       S0   S1   S2   S3   S4   S5   S6   S7   S8   S9   │
               │    │    │    │    │    │    │    │    │    │    │
               └────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘
                            Accumulate in batch
                                                                   │
                                                                   ▼
                                                          BLE Packet Sent
                                                          (10 samples, 161 bytes)

Packet Rate: 1000 samples/sec ÷ 10 samples/packet = 100 packets/sec
```

---

### Step 4: BLE Batch Transmission

```
┌──────────────────────────────────────────────────────────┐
│              Batch Transmission System                    │
├──────────────────────────────────────────────────────────┤
│                                                            │
│  current_ble_packet (161 bytes max):                      │
│  ┌────────────────────────────────────────────┐          │
│  │ [0] sample_count = 10                      │          │
│  │ [1-16] Sample 0: {local[4], remote[4]}    │          │
│  │ [17-32] Sample 1: {local[4], remote[4]}   │          │
│  │ [33-48] Sample 2: {local[4], remote[4]}   │          │
│  │ ...                                         │          │
│  │ [145-160] Sample 9: {local[4], remote[4]} │          │
│  └────────────────────────────────────────────┘          │
│          │                                                 │
│          │ When full OR 10ms elapsed                      │
│          ▼                                                 │
│  ┌────────────────────┐                                   │
│  │ send_current_batch()│                                  │
│  ├────────────────────┤                                   │
│  │ 1. Set sample_count │                                  │
│  │ 2. Calculate size   │                                  │
│  │ 3. setValue()       │                                  │
│  │ 4. notify()         │                                  │
│  │ 5. Reset counter    │                                  │
│  └────────┬───────────┘                                   │
│           │ BLE Notification                              │
│           ▼                                                │
│  ┌────────────────────┐                                   │
│  │ BLE Stack          │                                   │
│  │ - Queue packet     │                                   │
│  │ - Send over air    │                                   │
│  └────────┬───────────┘                                   │
│           │ ~129 kbps                                     │
└───────────┼──────────────────────────────────────────────┘
            │
            ▼
  ┌─────────────────────┐
  │  BLE Client         │
  │  - Notification RX  │
  │  - Parse packet     │
  │  - Extract samples  │
  └─────────────────────┘
```

---

## Command Flow: BLE Client → ESP32 RX Radio → Teensy

```
BLE Client Application
         │
         │ Write command string (UTF-8)
         │ e.g., "ALL_START"
         ▼
┌────────────────────────┐
│ Command Characteristic │ UUID: 11111111-2222-3333-4444-555555555555
│ (Write/Write-No-Resp)  │
└────────┬───────────────┘
         │ onWrite() callback
         ▼
┌─────────────────────────┐
│ MyCommandCallbacks      │
│  - Extract string       │
│  - Call process_ble_command() │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────┐
│ process_ble_command()                           │
├─────────────────────────────────────────────────┤
│ 1. Trim & uppercase command                     │
│ 2. Match against command list                   │
│ 3. Forward to ESP32 RX Radio via Serial2 UART   │
│ 4. Wait for response (timeout based on command) │
│    - Standard commands: 5 sec                   │
│    - ZERO commands: 15 sec                      │
│ 5. Read and log response                        │
└────────┬────────────────────────────────────────┘
         │ UART TX (921600 bps)
         ▼
┌───────────────────────┐
│ ESP32 RX Radio        │
│  - Receives command   │
│  - Parses and routes: │
│    • Local Teensy     │───► UART to Teensy 4.1 (Local)
│    • Remote Teensy    │───► Radio to Teensy 4.1 (Remote)
│    • Both Teensys     │───► UART + Radio
│  - Sends response     │
└───────────────────────┘
```

### Command Types and Routing

```
┌────────────────────────────────────────────────────────────────┐
│                      Command Categories                         │
├────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. LOCAL COMMANDS (START, STOP, RESET, etc.)                  │
│     BLE Client → BLE Slave → ESP32 RX → UART → Local Teensy   │
│     Response time: < 1-3 seconds                                │
│                                                                  │
│  2. REMOTE COMMANDS (REMOTE_START, REMOTE_STOP, etc.)          │
│     BLE Client → BLE Slave → ESP32 RX → Radio → Remote Teensy │
│     Response time: < 2-4 seconds (wireless delay)              │
│                                                                  │
│  3. DUAL COMMANDS (ALL_START, ALL_STOP, etc.)                  │
│     BLE Client → BLE Slave → ESP32 RX → UART + Radio → Both   │
│     Response time: < 2-4 seconds                                │
│                                                                  │
│  4. ZERO COMMANDS (ZERO, REMOTE_ZERO, ALL_ZERO)                │
│     Same routing as above, but longer processing time           │
│     Response time: 10-20 seconds (calibration averaging)        │
│                                                                  │
│  5. ESP32 COMMANDS (LOCAL_ON/OFF, REMOTE_ON/OFF, STATUS)       │
│     BLE Client → BLE Slave → ESP32 RX (handled locally)        │
│     Response time: < 1 second                                   │
│                                                                  │
└────────────────────────────────────────────────────────────────┘
```

---

## Data Conversion Pipeline

### Int32 → Int16 Conversion

```
UART Packet                    Redis Store                BLE Packet
─────────────                  ────────────               ──────────

lc[4] (int32_t)               lc[4] (int32_t)           local_lc[4] (int16_t)
each: 4 bytes                 each: 4 bytes             each: 2 bytes
range: ±2,147,483,647         range: ±2,147,483,647     range: ±32,767

Example value: 12,345         Example value: 12,345     Converted: 12,345
         │                            │                          │
         │ No conversion              │ Conversion               │
         ▼                            ▼                          ▼
   [45, 30, 00, 00]             [45, 30, 00, 00]          [45, 30]
   (little-endian)              (little-endian)           (little-endian)

Saturation/Clamping:
   if (value > 32767)  → value = 32767   (max int16)
   if (value < -32768) → value = -32768  (min int16)
```

**Why int16?**
- Load cell ADC values typically fit in ±32K range
- Reduces BLE packet size by 50%
- Doubles throughput (10 samples/packet instead of 5)
- Still maintains sufficient precision for force measurements

---

## Performance Metrics

### Data Throughput

```
┌─────────────────────────────────────────────────────────────┐
│                    Throughput Analysis                       │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Sample Rate:        1000 samples/sec (per 8-channel set)   │
│  Packet Rate:        100 packets/sec                         │
│  Samples/Packet:     10 samples                              │
│  Packet Size:        161 bytes (1 + 10×16)                   │
│  Data Rate:          16,100 bytes/sec                        │
│  Bandwidth:          128.8 kbps                              │
│                                                               │
│  Per Second Breakdown:                                        │
│  ├─ 1000 timer interrupts                                    │
│  ├─ 1000 Redis queue reads                                   │
│  ├─ 1000 int32→int16 conversions                             │
│  ├─ 100 BLE notifications sent                               │
│  └─ 8000 load cell values transmitted                        │
│     (1000 × 8 channels)                                       │
│                                                               │
│  Latency:                                                     │
│  ├─ UART RX → Redis: < 1 ms                                  │
│  ├─ Redis → BLE packet: < 10 ms (batch window)               │
│  ├─ BLE transmission: 5-20 ms (connection interval)          │
│  └─ End-to-end: ~15-30 ms typical                            │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Memory Layout

### BLE Packet Memory Structure

```
Byte Index  │ Content                    │ Data Type     │ Value Example
────────────┼────────────────────────────┼───────────────┼──────────────
0           │ sample_count               │ uint8_t       │ 10 (0x0A)
────────────┼────────────────────────────┼───────────────┼──────────────
1-2         │ samples[0].local_lc[0]     │ int16_t (LE)  │ 1234 (0xD2 0x04)
3-4         │ samples[0].local_lc[1]     │ int16_t (LE)  │ 2345 (0x29 0x09)
5-6         │ samples[0].local_lc[2]     │ int16_t (LE)  │ -500 (0x0C 0xFE)
7-8         │ samples[0].local_lc[3]     │ int16_t (LE)  │ 890 (0x7A 0x03)
9-10        │ samples[0].remote_lc[0]    │ int16_t (LE)  │ 5678 (0x2E 0x16)
11-12       │ samples[0].remote_lc[1]    │ int16_t (LE)  │ 6789 (0x85 0x1A)
13-14       │ samples[0].remote_lc[2]    │ int16_t (LE)  │ 7890 (0xD2 0x1E)
15-16       │ samples[0].remote_lc[3]    │ int16_t (LE)  │ 8901 (0xC5 0x22)
────────────┼────────────────────────────┼───────────────┼──────────────
17-32       │ samples[1]                 │ 16 bytes      │ ...
33-48       │ samples[2]                 │ 16 bytes      │ ...
...         │ ...                        │ ...           │ ...
145-160     │ samples[9]                 │ 16 bytes      │ ...
────────────┴────────────────────────────┴───────────────┴──────────────

Little-Endian Example:
  Value: 1234 (decimal) = 0x04D2 (hex)
  Byte[0] = 0xD2 (LSB)
  Byte[1] = 0x04 (MSB)
```

---

## Task Priority and Core Assignments

```
┌─────────────────────────────────────────────────────────────┐
│                ESP32 Dual-Core Task Distribution             │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Core 0 (Protocol CPU):                                      │
│  ┌──────────────────────────────────────────────┐           │
│  │ Task: uart_rx_task                           │           │
│  │ Priority: 23 (high)                          │           │
│  │ Stack: 8 KB                                  │           │
│  │ Function: Receive and parse UART packets     │           │
│  │           Store in Redis queues              │           │
│  └──────────────────────────────────────────────┘           │
│  ┌──────────────────────────────────────────────┐           │
│  │ Arduino Core Tasks (BLE, WiFi, etc.)         │           │
│  │ Priority: 1-10 (low-medium)                  │           │
│  └──────────────────────────────────────────────┘           │
│                                                               │
│  Core 1 (Application CPU):                                   │
│  ┌──────────────────────────────────────────────┐           │
│  │ Task: timer_processing_task                  │           │
│  │ Priority: 24 (highest)                       │           │
│  │ Stack: 4 KB                                  │           │
│  │ Function: Process timer interrupts           │           │
│  │           Create 8-channel samples           │           │
│  │           Build BLE packets                  │           │
│  └──────────────────────────────────────────────┘           │
│  ┌──────────────────────────────────────────────┐           │
│  │ Task: ble_tx_task                            │           │
│  │ Priority: 22 (high)                          │           │
│  │ Stack: 4 KB                                  │           │
│  │ Function: Send partial batches               │           │
│  │           Periodic flush                     │           │
│  └──────────────────────────────────────────────┘           │
│  ┌──────────────────────────────────────────────┐           │
│  │ Task: loop() (Arduino main)                  │           │
│  │ Priority: 1 (lowest)                         │           │
│  │ Function: Print stats, handle commands       │           │
│  └──────────────────────────────────────────────┘           │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## Error Handling Flow

```
┌──────────────────────────────────────────────────────────────┐
│                      Error Detection                          │
├──────────────────────────────────────────────────────────────┤
│                                                                │
│  UART Reception:                                               │
│  ┌────────────────────────────────┐                           │
│  │ Sync Error (0xAA 0x55 missing) │→ uart_sync_errors++       │
│  │ CRC Error (checksum mismatch)  │→ uart_crc_errors++        │
│  │ → Discard packet, wait for next sync                       │
│  └────────────────────────────────┘                           │
│                                                                │
│  Redis Queue:                                                  │
│  ┌────────────────────────────────┐                           │
│  │ Queue Full (>20 samples)       │→ Overwrite oldest         │
│  │ No data available              │→ Use last_known fallback  │
│  └────────────────────────────────┘                           │
│                                                                │
│  BLE Transmission:                                             │
│  ┌────────────────────────────────┐                           │
│  │ notify() throws exception      │→ ble_send_errors++        │
│  │ Client disconnected            │→ Reset batch counter      │
│  │ → Continue operation, retry next packet                    │
│  └────────────────────────────────┘                           │
│                                                                │
│  Commands:                                                     │
│  ┌────────────────────────────────┐                           │
│  │ Unknown command                │→ ble_command_errors++     │
│  │ Timeout waiting for response   │→ command_timeouts++       │
│  │ → Log error, continue operation                            │
│  └────────────────────────────────┘                           │
│                                                                │
└──────────────────────────────────────────────────────────────┘
```

---

## State Machine Diagram

### BLE Connection State

```
     ┌─────────────────┐
     │ Power On / Reset│
     └────────┬────────┘
              │
              ▼
     ┌─────────────────┐
     │  BLE Advertising │◄──────────┐
     │  (Discoverable)  │           │
     └────────┬─────────┘           │
              │ Client Connect       │
              ▼                      │
     ┌─────────────────┐            │
     │   Connected      │            │
     │  (No Data Flow)  │            │
     └────────┬─────────┘            │
              │ Client enables        │
              │ notifications         │
              ▼                      │
     ┌─────────────────┐            │
     │  Data Streaming  │            │
     │  (100 pps)       │            │
     └────────┬─────────┘            │
              │ Client Disconnect     │
              └───────────────────────┘
```

### Command Execution State

```
  ┌──────────┐
  │   Idle   │
  └────┬─────┘
       │ BLE Write received
       ▼
  ┌──────────────────┐
  │ Parse Command    │
  └────┬─────────────┘
       │
       ├─── Known command ───► Forward to ESP32 RX Radio (UART)
       │                           │
       │                           ▼
       │                      ┌──────────────────┐
       │                      │ Wait for Response│
       │                      │ (1-20 sec timeout)│
       │                      └────┬─────────────┘
       │                           │
       │                           ├─ Response received ──► Log & Done
       │                           └─ Timeout ──────────► Log timeout
       │
       └─── Unknown command ───► Log error & Done
```

---

## Synchronization Mechanism

### Preventing Duplicate Samples

```
Problem: Same UART sample consumed multiple times by 1000 Hz timer

Solution: "consumed" flag in Redis queue

┌────────────────────────────────────────────────────────────┐
│                    Sample Lifecycle                         │
├────────────────────────────────────────────────────────────┤
│                                                              │
│  1. UART receives packet                                    │
│     └─► add_to_redis_queue()                                │
│         └─► sample.consumed = false                         │
│                                                              │
│  2. Timer interrupt (1000 Hz)                               │
│     └─► get_next_redis_sample()                             │
│         ├─► Check: if (!sample.consumed)                    │
│         ├─► Copy data                                        │
│         ├─► sample.consumed = true                          │
│         ├─► Remove from queue                               │
│         └─► Update last_known fallback                      │
│                                                              │
│  3. Next timer interrupt                                    │
│     └─► get_next_redis_sample()                             │
│         ├─► Previous sample already consumed                │
│         ├─► Try next sample in queue                        │
│         └─► If none available: use last_known fallback      │
│                                                              │
│  Result: Each UART sample consumed ONCE, gaps filled with   │
│          last known value (hold-over)                       │
│                                                              │
└────────────────────────────────────────────────────────────┘
```

---

## Summary

This ESP32 BLE Load Cell Server implements a sophisticated data flow:

1. **UART Reception** (400 pps, 24-byte packets) → Redis queues
2. **Timer-Based Sampling** (1000 Hz) → Combines local + remote
3. **Batch Transmission** (100 pps, 161-byte packets) → BLE client
4. **Bidirectional Commands** → Full control over Teensy boards

**Key Features:**
- Zero-copy data paths where possible
- Dual-core task distribution for performance
- Redis-like queue prevents duplicates
- Optimized int16 format (50% size reduction)
- Robust error handling and statistics

**Supported Use Cases:**
- Real-time force plate monitoring
- Gait analysis systems
- Balance and stability testing
- Research data collection
- Remote system control via BLE

---

**Document Version:** 1.0  
**Last Updated:** October 2025  
**Compatible with:** ESP32 BLE Slave Firmware v1.0


