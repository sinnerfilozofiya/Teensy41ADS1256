# Force Plate Calibration - Quick Reference

## 🚀 Quick Start Calibration Sequence

### 1. Initial Setup
```
HELP                    # Show all available commands
CAL_SUMMARY            # Check current calibration status
```

### 2. Step A: ADC Calibration (5 minutes)
```
CAL_ADC                # Perform ADC self-calibration
```
✅ **Result**: ADC calibrated for current PGA/data rate settings

### 3. Step B: Load Cell Calibration (30 minutes)

#### Tare All Load Cells (no load on plate)
```
CAL_TARE_1             # Tare load cell 1
CAL_TARE_2             # Tare load cell 2  
CAL_TARE_3             # Tare load cell 3
CAL_TARE_4             # Tare load cell 4
```

#### Span Calibration (for each load cell)
```
CAL_SPAN_1             # Start span calibration for LC1

# Add calibration points (place masses directly above LC1)
CAL_SPAN_ADD 1 0.000   # No load
CAL_SPAN_ADD 1 5.000   # 5kg load
CAL_SPAN_ADD 1 10.000  # 10kg load
CAL_SPAN_ADD 1 15.000  # 15kg load
CAL_SPAN_ADD 1 20.000  # 20kg load

CAL_SPAN_COMPUTE 1     # Compute span coefficients
```
🔄 **Repeat for load cells 2, 3, and 4**

### 4. Step C: Matrix Calibration (20 minutes)
```
CAL_MATRIX_START       # Start matrix calibration

# Add calibration points (place 10kg at various positions)
CAL_MATRIX_ADD 10.000 0.0 0.0      # Center
CAL_MATRIX_ADD 10.000 0.0 150.0    # Front center
CAL_MATRIX_ADD 10.000 0.0 -150.0   # Back center
CAL_MATRIX_ADD 10.000 150.0 0.0    # Right center
CAL_MATRIX_ADD 10.000 -150.0 0.0   # Left center
CAL_MATRIX_ADD 10.000 150.0 150.0  # Front-right corner
CAL_MATRIX_ADD 10.000 -150.0 150.0 # Front-left corner
CAL_MATRIX_ADD 10.000 150.0 -150.0 # Back-right corner
CAL_MATRIX_ADD 10.000 -150.0 -150.0# Back-left corner

CAL_MATRIX_COMPUTE     # Compute matrix coefficients
```

### 5. Save and Verify
```
CAL_SAVE               # Save calibration to EEPROM
CAL_SUMMARY            # Verify all calibrations are valid
CAL_FORCE_DATA         # Test with known mass
```

---

## 📋 Command Reference

### Basic Commands
| Command | Description |
|---------|-------------|
| `HELP` | Show all available commands |
| `CAL_SUMMARY` | Show calibration status |
| `CAL_LOAD` | Load calibration from EEPROM |
| `CAL_SAVE` | Save calibration to EEPROM |
| `CAL_CLEAR` | Clear all calibration data |

### Step A: ADC Calibration
| Command | Description |
|---------|-------------|
| `CAL_ADC` | Perform ADC self-calibration |

### Step B: Load Cell Calibration
| Command | Description |
|---------|-------------|
| `CAL_TARE_<1-4>` | Tare individual load cell |
| `CAL_SPAN_<1-4>` | Start span calibration |
| `CAL_SPAN_ADD <ch> <kg>` | Add span calibration point |
| `CAL_SPAN_COMPUTE <ch>` | Compute span coefficients |
| `CAL_SHUNT_<1-4>` | Perform shunt calibration |

### Step C: Matrix Calibration
| Command | Description |
|---------|-------------|
| `CAL_MATRIX_START` | Start matrix calibration |
| `CAL_MATRIX_ADD <kg> <x> <y>` | Add matrix calibration point |
| `CAL_MATRIX_COMPUTE` | Compute matrix coefficients |

### Measurements
| Command | Description |
|---------|-------------|
| `CAL_FORCE_DATA` | Show calibrated force/COP data |
| `SHOW_VALUES` | Show raw load cell readings |

### Validation & Testing
| Command | Description |
|---------|-------------|
| `VAL_START` | Start validation system |
| `VAL_ADD_TEST <name> <kg> <x> <y>` | Add validation test |
| `VAL_RUN_TESTS` | Run all validation tests |
| `VAL_SHOW_RESULTS` | Show validation results |
| `VAL_REPEATABILITY <kg> <x> <y> <n>` | Run repeatability test |

### Diagnostics
| Command | Description |
|---------|-------------|
| `DIAG_LC` | Load cell diagnostics |
| `DIAG_ADC` | ADC diagnostics |

---

## 🎯 Typical Calibration Session

### Equipment Needed
- ✅ Calibrated masses: 5kg, 10kg, 15kg, 20kg
- ✅ Measuring tape/ruler
- ✅ Level surface
- ✅ Serial monitor connection

### Session Timeline
1. **Setup** (5 min): Connect, check status
2. **ADC Cal** (5 min): `CAL_ADC`
3. **Tare** (10 min): `CAL_TARE_1` through `CAL_TARE_4`
4. **Span** (30 min): Individual load cell calibration
5. **Matrix** (20 min): Whole-plate calibration
6. **Save** (2 min): `CAL_SAVE`, verify
7. **Test** (10 min): Validation tests

**Total Time: ~1.5 hours**

---

## ⚠️ Troubleshooting

### Common Issues

**"No valid calibration data"**
```
CAL_CLEAR              # Clear corrupted data
# Redo complete calibration sequence
CAL_SAVE               # Save when complete
```

**High linearity errors (>0.5%)**
```
DIAG_LC                # Check load cell health
# Use more calibration points
# Ensure masses placed directly above load cells
```

**Inconsistent COP readings**
```
CAL_MATRIX_START       # Redo matrix calibration
# Use more positions and masses
# Verify position measurements
```

**ADC calibration fails**
```
DIAG_ADC               # Check ADC status
# Check connections and power supply
# Allow longer settling time
```

### Quality Checks

**After each step, verify:**
- ✅ No error messages
- ✅ Reasonable coefficient values
- ✅ Low linearity errors (<0.1%)
- ✅ Stable readings

**Final validation:**
```
VAL_START
VAL_ADD_TEST center 10.000 0.0 0.0
VAL_ADD_TEST corner 10.000 100.0 100.0
VAL_RUN_TESTS
VAL_SHOW_RESULTS
```

---

## 📊 Expected Results

### Good Calibration Indicators
- **Force accuracy**: <1% error
- **COP accuracy**: <3mm error  
- **Linearity**: <0.1% error
- **Repeatability**: <0.5% CV
- **Zero stability**: <50 counts drift

### Calibration Certificates
Document for each calibration:
- Date and operator
- Masses used (with certificates)
- Environmental conditions
- Calibration results
- Next calibration due date

---

## 🔄 Maintenance Schedule

### Daily (if in use)
```
CAL_FORCE_DATA         # Quick functionality check
```

### Weekly
```
CAL_TARE_1             # Check zero stability
CAL_TARE_2
CAL_TARE_3  
CAL_TARE_4
```

### Monthly
```
VAL_REPEATABILITY 10.000 0.0 0.0 5  # Repeatability check
```

### Annually
```
# Complete recalibration sequence
# Update calibration certificates
```

---

## 🚨 Emergency Procedures

### System Reset
```
CAL_CLEAR              # Clear all calibration
RESET                  # Reset Teensy
# Redo complete calibration
```

### Backup Calibration
```
CAL_SUMMARY            # Document current calibration
# Save output to file for backup
```

### Quick Validation
```
DIAG_LC                # Check load cell health
DIAG_ADC               # Check ADC health
CAL_FORCE_DATA         # Test with known mass
```
