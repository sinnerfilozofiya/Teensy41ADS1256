# Teensy Calibration (Per-Load-Cell, Auto-Fit Regression)

This project implements a **Teensy-only** calibration layer for a 4-load-cell force plate read by an **ADS1256**. It is designed to be:

- **Easy to operate** from the Serial Monitor
- **Stable on Teensy 4.1** (no heap use in calibration math; no packed-float structs)
- **Non-invasive** to the high-rate data path (**does not change the frame/stream output format**)

---

## Goals / Key Decisions

- **Per-load-cell calibration**: each load cell gets its own calibration curve (not a single “total plate” scale).
- **No intercept**: calibration is fit **through zero** so when delta-counts are zero, mass is zero.
- **Units**: calibrated outputs are in **10 g units** (0.01 kg).
  - \(1\ \text{kg} = 100\) units
  - Max total (4 × 200 kg = 800 kg) → 80,000 units → **requires int32** for totals.
- **Auto-fit**: after every `CAL ADD ...`, the code immediately performs the fit and saves it.
- **Streaming untouched**: raw samples are still packed as **int24** in the existing frame.

---

## Files

- `ads_1256_custom_library/ads_1256_custom_library.ino`
  - Command handler for Serial Monitor (`CAL ...` commands)
  - Acquisition + frame packing (unchanged by calibration)
- `ads_1256_custom_library/calibration_regression.h`
- `ads_1256_custom_library/calibration_regression.cpp`
  - Calibration storage (EEPROM) + capture + point management + regression fit

---

## How Calibration Works (Conceptually)

### 1) Tare (offsets)
`CAL TARE` measures “no load” baseline counts for each load cell and stores:

- `offsets[ch]` (ADC counts at zero load)

Implementation notes:
- Captures **3 windows** (default **600 ms** each) and selects the most stable one (lowest summed stddev).
- Uses filtered reads by default (can use raw with `CAL READ RAW` for debugging).

### 2) Add a known load point (per-cell points derived from total load)
`CAL ADD <kg>` captures the plate under a known **total** weight and creates **one calibration point per cell**:

- Compute per-cell delta counts: \(\Delta_c = |mean_c - offset_c|\)
- Compute sum: \(S = \sum \Delta_c\)
- Known total output in 10g units: \(Y = kg \times 100\)
- Assign each cell its share based on contribution:
  - \(Y_c = Y \times (\Delta_c / S)\)
- Store point for that cell: \((x=\Delta_c,\ y=Y_c)\)

Why this approach:
- With only **total** applied weight and unknown distribution, you cannot directly know each cell’s “true” weight.
- The **share-by-contribution** method gives each cell consistent points for per-cell fitting and works well for symmetric/typical loading patterns.

Important guard:
- If a channel is saturated (near ADS1256 rail), it is **skipped** so it doesn’t corrupt calibration.

### 3) Fit (automatic after each `CAL ADD`)
After adding points, the code fits **per-cell** scale using a **through-zero least squares** model:

- Model per cell: \(y = a x\)
  - \(x\): absolute delta counts for that cell
  - \(y\): assigned 10g units for that cell
- **1 point** → “v1” behavior (simple linear scale through zero)
- **2+ points** → regression fit through zero

The result stored is only:
- `ch_a_10g_per_count[ch]`

Total mass is computed as:
- `TOTAL = sum(LC1..LC4)`

---

## Commands (Serial Monitor @ 115200)

Core:
- `CAL CLEAR`  
  Clears calibration in EEPROM (resets offsets/scales/points).
- `CAL TARE`  
  Captures offsets at no load.
- `CAL ADD <kg>`  
  Adds a multi-load point using total weight and **auto-fits + saves** immediately.
- `CAL SHOW`  
  Prints offsets, per-cell fitted slopes, and point counts.
- `CAL POINTS`  
  Prints stored points per load cell.
- `CAL READ`  
  Prints per-cell + total in 10g units (uses filtered reads).
- `CAL READ RAW`  
  Same, but uses raw ADC reads.

Optional (advanced / controlled loading):
- `CAL ADD_CH <1-4> <kg>`  
  Adds a point to one specific channel (only useful if you can load that cell predominantly). Auto-fits + saves.

---

## Recommended Workflow

1. `CAL CLEAR`
2. Ensure plate is unloaded and still → `CAL TARE`
3. Apply known weights, low → high (example):
   - apply 5 kg → `CAL ADD 5`
   - apply 10 kg → `CAL ADD 10`
   - apply 40 kg → `CAL ADD 40`
4. `CAL SHOW`
5. `CAL READ` and confirm totals make sense.

---

## Known Pitfalls / Debug Tips

### ADS1256 rail / broken channel
If you see an offset like:
- `offset=8388607` (exactly \(2^{23}-1\))

That channel is **saturated/railed** (wiring, mux mapping, open input, polarity, etc.). Calibration will be unreliable until fixed.

Use:
- `SHOW_FILTERED` (existing debug command) to see raw/filtered counts.

### Why Teensy previously rebooted during FIT
Using `__attribute__((packed))` on a struct that contains **floats** can misalign float fields on Cortex-M7, causing a **hardfault** (reboot) when accessing them.  
The current calibration code stores calibration in a properly aligned struct and uses `.cpp`/`.h` modules to avoid Arduino `.ino` auto-prototype issues.

---

## BLE Control (Wireless Calibration)

The system supports full calibration control via BLE using the ESP32 BLE Slave.

### BLE Characteristics

| UUID | Name | Type | Description |
|------|------|------|-------------|
| `12345678-1234-1234-1234-123456789abc` | Service | - | Main BLE service |
| `87654321-4321-4321-4321-cba987654321` | Data | NOTIFY | Binary sensor data (8 channels, 16-bit) |
| `11111111-2222-3333-4444-555555555555` | Cmd | WRITE + NOTIFY | Send commands, receive JSON responses |

### BLE Commands

Prefix commands with `LOCAL_` or `REMOTE_` to target specific Teensy:

| Command | Description |
|---------|-------------|
| `LOCAL_CAL_TARE` | Tare local Teensy |
| `LOCAL_CAL_SHOW` | Show local calibration |
| `LOCAL_CAL_ADD_10` | Add 10kg point to local |
| `LOCAL_CAL_CLEAR` | Clear local calibration |
| `LOCAL_CAL_READ` | Read calibrated values |
| `REMOTE_CAL_TARE` | Tare remote Teensy |
| `REMOTE_CAL_SHOW` | Show remote calibration |
| `LOCAL_PING` | Ping local Teensy |
| `REMOTE_PING` | Ping remote Teensy |
| `START` / `STOP` | Control data acquisition |

### JSON Response Format

All responses are sent as JSON via the Response characteristic:

```json
// PING response
{"target":"LOCAL","cmd":"PING","ok":true,"ms":45}

// TARE response
{"target":"LOCAL","cmd":"TARE","ok":true,"ms":650}

// SHOW response
{"target":"LOCAL","cmd":"SHOW","lc":[
  {"off":38853,"a":0.015743,"n":2},
  {"off":12345,"a":0.014892,"n":2},
  {"off":-1234,"a":0.016021,"n":2},
  {"off":-5678,"a":0.015156,"n":2}
],"ms":52}

// READ response
{"target":"LOCAL","cmd":"READ","v":[100,150,120,130,500],"ms":35}

// Error response
{"target":"LOCAL","cmd":"TARE","ok":false,"err":"TIMEOUT","ms":5001}
```

### Testing with nRF Connect (Android/iOS)

1. **Connect**: Open nRF Connect, scan for "LoadCell_BLE_Server", connect.

2. **Subscribe to Cmd characteristic**: Find service `12345678...`, expand.
   - Tap the Cmd characteristic (`11111111...`)
   - Enable notifications (arrow icon) - this is where responses come back

3. **Send Command**: 
   - Same Cmd characteristic (`11111111...`)
   - Write text: `LOCAL_PING` and send
   - Response appears as notification: `{"target":"LOCAL","cmd":"PING","ok":true,"ms":XX}`

4. **Calibration Workflow**:
   ```
   Send: LOCAL_CAL_CLEAR     → {"target":"LOCAL","cmd":"CLEAR","ok":true,...}
   Send: LOCAL_CAL_TARE      → {"target":"LOCAL","cmd":"TARE","ok":true,...}
   Send: LOCAL_CAL_ADD_10    → {"target":"LOCAL","cmd":"ADD","ok":true,...}
   Send: LOCAL_CAL_SHOW      → {"target":"LOCAL","cmd":"SHOW","lc":[...],...}
   ```

5. **Sensor Data Stream**: Enable notifications on Data characteristic (`87654321...`) to receive binary sensor data.

---

## Extension Ideas (Future Development)

- **Piecewise linear calibration** per cell (more accurate across range) by fitting segments.
- **Stability gating**: require stddev below a threshold before accepting `TARE`/`ADD`.
- **Persist points vs only persist fit**: currently points are kept in RAM until saved; you can persist point history if needed.
- **COP estimation**: once per-cell forces are stable, compute approximate COP using geometry and calibrated forces.
- **Custom BLE App**: Build a Flutter/React Native app that uses the JSON responses for a polished UI.





