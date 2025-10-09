# Force Plate Calibration Guide

## Overview

This guide provides step-by-step instructions for calibrating your Teensy 4.1 + ADS1256 force plate system. The calibration follows a 3-step process based on industry standards for force platform calibration.

## Prerequisites

### Hardware Required
- Calibrated masses (deadweights): 1kg, 2kg, 5kg, 10kg, 20kg (or similar range)
- Precision shunt resistors (if using shunt calibration)
- Thermometer (for temperature compensation)
- Measuring tape/ruler for position measurements
- Level surface and stable mounting

### Software Setup
- Upload the complete calibration system to your Teensy 4.1
- Connect via Serial Monitor (115200 baud)
- Ensure EEPROM is available for calibration storage

## Step A: ADC (ADS1256) Calibration

### Purpose
Calibrate the ADS1256 ADC for your specific PGA and data rate settings to ensure accurate voltage measurements.

### Procedure

1. **Connect to Serial Monitor**
   ```
   Type: HELP
   ```

2. **Perform ADC Self-Calibration**
   ```
   Type: CAL_ADC
   ```
   
   This performs:
   - SELFOCAL (offset calibration)
   - SELFGCAL (gain calibration)
   - Stores OFC and FSC register values

3. **Verify Calibration**
   ```
   Type: CAL_SUMMARY
   ```
   
   Should show valid ADC calibration with OFC and FSC values.

### Notes
- Calibration takes ~5 seconds
- Performed automatically for PGA=64, DR=30kSPS
- For different settings, modify the `CAL_ADC` command parameters

## Step B: Per-Load-Cell Calibration

### Purpose
Calibrate each load cell individually for:
- Zero offset (tare)
- Span (sensitivity)
- Linearity
- Shunt calibration (optional)
- Temperature compensation (optional)

### B1: Zero Calibration (Tare)

**For each load cell (1-4):**

1. **Remove all loads** from the force plate
2. **Wait for thermal stability** (~10 minutes)
3. **Perform tare calibration:**
   ```
   Type: CAL_TARE_1    # For load cell 1
   Type: CAL_TARE_2    # For load cell 2
   Type: CAL_TARE_3    # For load cell 3
   Type: CAL_TARE_4    # For load cell 4
   ```

Each tare collects 500 samples and computes the average zero offset.

### B2: Span Calibration

**For each load cell individually:**

1. **Start span calibration:**
   ```
   Type: CAL_SPAN_1    # For load cell 1
   ```

2. **Add calibration points** (minimum 5 points recommended):
   
   **0% Load (no weight):**
   ```
   Type: CAL_SPAN_ADD 1 0.000
   ```
   
   **25% Load (place 5kg directly above load cell 1):**
   ```
   Type: CAL_SPAN_ADD 1 5.000
   ```
   
   **50% Load (place 10kg directly above load cell 1):**
   ```
   Type: CAL_SPAN_ADD 1 10.000
   ```
   
   **75% Load (place 15kg directly above load cell 1):**
   ```
   Type: CAL_SPAN_ADD 1 15.000
   ```
   
   **100% Load (place 20kg directly above load cell 1):**
   ```
   Type: CAL_SPAN_ADD 1 20.000
   ```

3. **Compute span calibration:**
   ```
   Type: CAL_SPAN_COMPUTE 1
   ```

4. **Repeat for all load cells** (2, 3, 4)

### B3: Shunt Calibration (Optional)

**For each load cell with shunt capability:**

1. **Connect precision shunt resistor** to load cell terminals
2. **Perform shunt calibration:**
   ```
   Type: CAL_SHUNT_1    # For load cell 1
   ```
3. **Follow on-screen prompts** to connect/disconnect shunt

### Validation
```
Type: CAL_SUMMARY
```
Should show valid calibration for all load cells with linearity errors < 0.1%.

## Step C: Matrix Calibration

### Purpose
Create a linear mapping from individual load cell outputs to:
- Total vertical force (Fz)
- Moments about X and Y axes (Mx, My)
- Center of pressure coordinates (COPx, COPy)

### C1: Data Collection

1. **Start matrix calibration:**
   ```
   Type: CAL_MATRIX_START
   ```

2. **Collect calibration points** at various positions:

   **Center position (0, 0):**
   ```
   Place 10kg at plate center
   Type: CAL_MATRIX_ADD 10.000 0.0 0.0
   ```
   
   **Front center (0, +150mm):**
   ```
   Place 10kg at front center
   Type: CAL_MATRIX_ADD 10.000 0.0 150.0
   ```
   
   **Back center (0, -150mm):**
   ```
   Place 10kg at back center  
   Type: CAL_MATRIX_ADD 10.000 0.0 -150.0
   ```
   
   **Right center (+150mm, 0):**
   ```
   Place 10kg at right center
   Type: CAL_MATRIX_ADD 10.000 150.0 0.0
   ```
   
   **Left center (-150mm, 0):**
   ```
   Place 10kg at left center
   Type: CAL_MATRIX_ADD 10.000 -150.0 0.0
   ```
   
   **Corner positions:**
   ```
   Type: CAL_MATRIX_ADD 10.000 150.0 150.0   # Front-right
   Type: CAL_MATRIX_ADD 10.000 -150.0 150.0  # Front-left  
   Type: CAL_MATRIX_ADD 10.000 150.0 -150.0  # Back-right
   Type: CAL_MATRIX_ADD 10.000 -150.0 -150.0 # Back-left
   ```

3. **Add multiple mass levels** at key positions:
   ```
   Type: CAL_MATRIX_ADD 5.000 0.0 0.0     # 5kg at center
   Type: CAL_MATRIX_ADD 15.000 0.0 0.0    # 15kg at center
   Type: CAL_MATRIX_ADD 20.000 0.0 0.0    # 20kg at center
   ```

### C2: Compute Matrix

```
Type: CAL_MATRIX_COMPUTE
```

This computes the linear transformation coefficients using least-squares fitting.

### C3: Validation

```
Type: CAL_FORCE_DATA
```

Place a known mass at a known position and verify:
- Force reading matches expected value (mass × 9.81)
- COP coordinates match actual position

## Final Steps

### Save Calibration
```
Type: CAL_SAVE
```

### Verify Complete Calibration
```
Type: CAL_SUMMARY
```

Should show:
- ✓ Valid ADC calibration
- ✓ Valid calibration for all 4 load cells
- ✓ Valid matrix calibration

## Usage After Calibration

### Real-time Force Data
```
Type: CAL_FORCE_DATA
```

Shows:
- Fz: Vertical force (N)
- Mx, My: Moments (N⋅m)
- COP: Center of pressure (mm)

### Integration with Data Acquisition
The calibrated force data is automatically applied during normal data acquisition when calibration is loaded.

## Troubleshooting

### Common Issues

**"No valid calibration data"**
- Perform complete calibration sequence
- Save calibration with `CAL_SAVE`

**High linearity errors (>0.5%)**
- Check load cell mounting and isolation
- Use more calibration points
- Ensure masses are placed directly above load cells

**Inconsistent COP readings**
- Verify matrix calibration with multiple positions
- Check plate geometry measurements
- Ensure proper load cell spacing

**ADC calibration fails**
- Check ADS1256 connections
- Verify power supply stability
- Allow longer settling time

### Calibration Maintenance

**Regular Checks (Monthly):**
- Verify zero offsets: `CAL_TARE_1` through `CAL_TARE_4`
- Check with known mass at center position

**Full Recalibration (Annually):**
- Complete Step A-C calibration sequence
- Document calibration certificates and traceability

**Shunt Verification (Weekly):**
- Use `CAL_SHUNT_X` commands for quick health checks
- Compare with baseline shunt readings

## Advanced Features

### Temperature Compensation
- Monitor ambient temperature during calibration
- Apply temperature coefficients for high-precision applications

### Multiple PGA/Data Rate Settings
- Modify `perform_adc_self_calibration()` parameters
- Store multiple ADC calibrations for different configurations

### Custom Matrix Equations
- Modify matrix calibration algorithms for specific plate geometries
- Implement higher-order corrections for non-linear effects

## Calibration Certificates

Document the following for traceability:
- Calibration date and operator
- Masses used (with certificates)
- Environmental conditions
- Calibration results and uncertainties
- Next calibration due date

## References

- NIST Handbook 150-2G: Calibration of Force-Measuring Instruments
- ASTM E74: Standard Practice for Calibration of Force-Measuring Instruments
- ISO 376: Metallic materials — Calibration of force-proving instruments
