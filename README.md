# Teensy 4.1 to ADS1256 Wiring Diagram

## Pin Connections

| ADS1256 Pin | Function | Teensy 4.1 Pin | Notes |
|-------------|----------|----------------|-------|
| VDD         | Power    | 3.3V           | ADS1256 power supply |
| AVDD        | Analog Power | 3.3V      | Analog power supply |
| DGND        | Digital Ground | GND      | Digital ground |
| AGND        | Analog Ground | GND       | Analog ground |
| SCLK        | SPI Clock | Pin 13 (SCK)  | SPI clock signal |
| DIN         | Data In   | Pin 11 (MOSI) | SPI data to ADS1256 |
| DOUT        | Data Out  | Pin 12 (MISO) | SPI data from ADS1256 |
| CS          | Chip Select | Pin 10      | SPI chip select (active low) |
| DRDY        | Data Ready | Pin 22       | Interrupt pin (active low) |
| RESET       | Reset     | Pin 8         | Reset pin (active low) |
| PDWN        | Power Down | 3.3V         | Tie high to keep powered |
| VREFP       | Positive Reference | External | Connect to your positive reference |
| VREFN       | Negative Reference | External | Connect to your negative reference |

## Changes from Teensy 3.1

- **CS Pin**: Changed from Pin 21 to Pin 10 (Teensy 4.1 default SPI CS)
- **All other pins remain the same**
- **Performance**: Teensy 4.1 runs at 600MHz vs 72MHz, providing much faster processing

## Power Supply Notes

- **ADS1256 operates at 3.3V** - Do NOT connect to 5V
- **Use clean power supply** - Consider adding decoupling capacitors (0.1µF ceramic + 10µF tantalum)
- **Separate analog and digital grounds** if possible for best noise performance

## Reference Voltage

- **VREFP/VREFN**: Determines the input voltage range
- **Common setup**: VREFP = +2.5V, VREFN = GND for 0-5V input range with gain
- **For ±2.5V inputs**: VREFP = +2.5V, VREFN = -2.5V

## Input Channels

The ADS1256 has 8 differential input channels:
- **AIN0-AIN7**: Analog input pins
- **AINCOM**: Common input for single-ended measurements

## Example Breadboard Layout

```
Teensy 4.1          ADS1256 Breakout
┌─────────────┐     ┌──────────────┐
│         3.3V├─────┤VDD      AIN0 │ ← Analog Input 0+
│          GND├─────┤DGND     AIN1 │ ← Analog Input 0-
│      Pin 13 ├─────┤SCLK     AIN2 │ ← Analog Input 1+
│      Pin 11 ├─────┤DIN      AIN3 │ ← Analog Input 1-
│      Pin 12 ├─────┤DOUT     AIN4 │ ← Analog Input 2+
│      Pin 10 ├─────┤CS       AIN5 │ ← Analog Input 2-
│      Pin 22 ├─────┤DRDY     AIN6 │ ← Analog Input 3+
│       Pin 8 ├─────┤RESET    AIN7 │ ← Analog Input 3-
│             │     │        VREFP│ ← +Reference (e.g., 2.5V)
│             │     │        VREFN│ ← -Reference (e.g., GND)
└─────────────┘     └──────────────┘
```

## Testing Your Setup

1. **Power Check**: Verify 3.3V on VDD and AVDD
2. **SPI Communication**: The code will print register values during initialization
3. **Data Ready**: Should see interrupt activity on DRDY pin
4. **ADC Values**: Should get reasonable readings based on your input signals

## Troubleshooting

- **No communication**: Check SPI wiring and CS pin
- **Noisy readings**: Check power supply decoupling and grounding
- **No DRDY interrupts**: Verify Pin 22 connection and pull-up if needed
- **Wrong values**: Check reference voltage connections and gain settings
