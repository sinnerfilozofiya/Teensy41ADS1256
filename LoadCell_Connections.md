# Load Cell Connections to ADS1256

## 4 Load Cell Setup

| Load Cell | ADS1256 Pins | Description |
|-----------|--------------|-------------|
| **Load Cell 1** | AIN0 (+), AIN1 (-) | Differential Channel 1 |
| **Load Cell 2** | AIN2 (+), AIN3 (-) | Differential Channel 2 |
| **Load Cell 3** | AIN4 (+), AIN5 (-) | Differential Channel 3 |
| **Load Cell 4** | AIN6 (+), AIN7 (-) | Differential Channel 4 |

## Load Cell Wiring

Each load cell typically has 4 wires:
- **Red**: Excitation + (connect to your reference voltage, e.g., 3.3V or 5V)
- **Black**: Excitation - (connect to GND)
- **White**: Signal + (connect to AINx+)
- **Green**: Signal - (connect to AINx-)

## Serial Output Format

Your code will now print:
```
LC1: [value]    LC2: [value]    LC3: [value]    LC4: [value]
```

Where each value is the raw ADC reading from each load cell.

## Notes

- All 4 channels are read sequentially in one cycle
- Values are signed 24-bit integers (-8,388,608 to +8,388,607)
- Positive/negative values depend on load direction and wiring
- You may need to calibrate each load cell individually for weight conversion
