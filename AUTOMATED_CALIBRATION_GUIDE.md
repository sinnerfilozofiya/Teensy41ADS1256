# 🤖 Automated Force Plate Calibration System

## Overview

The automated calibration system guides you through the complete 3-step force plate calibration process with clear instructions, timing, and user prompts. No need to remember complex command sequences!

## 🚀 Quick Start

### Start Automated Calibration
```
Type: AUTOMATED_CALIBRATION
```
or
```
Type: AUTO_CAL_START
```

### During Calibration
- **`CONTINUE`** - Proceed to next step
- **`SKIP`** - Skip current step  
- **`ABORT`** - Cancel calibration
- **`STATUS`** - Show progress

## 📋 What the System Does

### 🎯 **Step A: ADC Calibration (5 minutes)**
- **Automatic**: Performs SELFOCAL and SELFGCAL
- **User Action**: Just wait for completion
- **Result**: ADC calibrated for accurate voltage readings

### 🎯 **Step B: Load Cell Calibration (30-40 minutes)**

#### B1: Tare (Automatic)
- **System**: Zeros all 4 load cells automatically
- **User Action**: Ensure no loads on plate

#### B2: Span Calibration (Interactive)
For each load cell (1-4):
1. **System prompts**: "Place 0kg (no load)"
2. **User**: Remove all masses → Type `CONTINUE`
3. **System prompts**: "Place 5kg above Load Cell X"
4. **User**: Place 5kg mass → Type `CONTINUE`
5. **System prompts**: "Place 10kg above Load Cell X"
6. **User**: Place 10kg mass → Type `CONTINUE`
7. **System prompts**: "Place 15kg above Load Cell X"
8. **User**: Place 15kg mass → Type `CONTINUE`
9. **System prompts**: "Place 20kg above Load Cell X"
10. **User**: Place 20kg mass → Type `CONTINUE`
11. **System**: Computes span coefficients automatically

### 🎯 **Step C: Matrix Calibration (20-30 minutes)**

The system guides you through 9 positions with 10kg mass:

1. **Center** (0, 0 mm)
2. **Front Center** (0, +150 mm)
3. **Back Center** (0, -150 mm)
4. **Right Center** (+150, 0 mm)
5. **Left Center** (-150, 0 mm)
6. **Front-Right Corner** (+150, +150 mm)
7. **Front-Left Corner** (-150, +150 mm)
8. **Back-Right Corner** (+150, -150 mm)
9. **Back-Left Corner** (-150, -150 mm)

For each position:
- **System prompts**: Clear position description
- **User**: Place 10kg mass → Type `CONTINUE`
- **System**: Records data automatically

### 🎯 **Save & Verify (Automatic)**
- **System**: Saves to EEPROM automatically
- **System**: Shows calibration summary
- **System**: Tests calibrated measurements

## 💬 Example Session

```
[AUTO-CAL] 🚀 AUTOMATED FORCE PLATE CALIBRATION
[AUTO-CAL] ⏱️  ESTIMATED TIME: 60-90 minutes
[AUTO-CAL] Ready to begin? Type 'CONTINUE' to start...

> CONTINUE

[AUTO-CAL] 📋 STEP A: ADC (ADS1256) CALIBRATION
[AUTO-CAL] ⏱️  DURATION: ~30 seconds
[AUTO-CAL] Ready to start ADC calibration?
[AUTO-CAL] Type 'CONTINUE' to proceed...

> CONTINUE

[AUTO-CAL] 🔄 Starting ADC self-calibration...
[AUTO-CAL] ✅ ADC calibration completed successfully!

[AUTO-CAL] 📋 STEP B: LOAD CELL CALIBRATION
[AUTO-CAL] ⚠️  Remove ALL loads from the plate before starting
[AUTO-CAL] Ready to start load cell calibration?
[AUTO-CAL] Type 'CONTINUE' to proceed...

> CONTINUE

[AUTO-CAL] 🔄 Taring all 4 load cells...
[AUTO-CAL] ✅ All load cells tared successfully!

[AUTO-CAL] 🔄 LOAD CELL 1 SPAN CALIBRATION
[AUTO-CAL] 📦 MASS POINT 1/5: 0.0 kg
[AUTO-CAL] 🔄 Remove all masses from Load Cell 1 area
[AUTO-CAL] Wait for reading to stabilize, then type 'CONTINUE'...

> CONTINUE

[AUTO-CAL] 📦 MASS POINT 2/5: 5.0 kg
[AUTO-CAL] 🔄 Place 5.0 kg directly above Load Cell 1
[AUTO-CAL] Wait for reading to stabilize, then type 'CONTINUE'...

> CONTINUE

... (continues for all mass points and all load cells)

[AUTO-CAL] 📋 STEP C: MATRIX CALIBRATION
[AUTO-CAL] 📍 POSITION 1/9: Center
[AUTO-CAL] 📦 Place 10kg mass at coordinates: (0, 0) mm
[AUTO-CAL] 🎯 CENTER: Place mass at the exact center of the plate
[AUTO-CAL] Wait for reading to stabilize, then type 'CONTINUE'...

> CONTINUE

... (continues for all positions)

[AUTO-CAL] 💾 SAVING AND VERIFICATION
[AUTO-CAL] ✅ Calibration data saved successfully!
[AUTO-CAL] 🎉 CALIBRATION COMPLETE!
```

## 🛠️ Equipment Needed

### Required Masses
- **5kg** calibrated mass
- **10kg** calibrated mass  
- **15kg** calibrated mass
- **20kg** calibrated mass

### Tools
- **Measuring tape** for position verification
- **Level surface** for stable measurements
- **Patience** - allow readings to stabilize

## ⏱️ Time Estimates

| Step | Duration | Description |
|------|----------|-------------|
| **Step A** | 5 minutes | ADC calibration (automatic) |
| **Step B** | 30-40 minutes | Load cell calibration (4 cells × 5 points) |
| **Step C** | 20-30 minutes | Matrix calibration (9 positions) |
| **Save/Verify** | 5 minutes | Automatic save and verification |
| **Total** | **60-90 minutes** | Complete calibration |

## 🎯 Key Features

### ✅ **Guided Process**
- Clear step-by-step instructions
- No need to remember command sequences
- Built-in help and status commands

### ✅ **Flexible Control**
- **CONTINUE** - Normal progression
- **SKIP** - Skip problematic steps
- **ABORT** - Cancel anytime
- **STATUS** - Check progress

### ✅ **Error Handling**
- Automatic retry options
- Skip capability for failed steps
- Clear error messages and recovery

### ✅ **Progress Tracking**
- Shows current step and progress
- Indicates which load cell/position
- Estimates remaining time

### ✅ **Quality Assurance**
- Automatic data validation
- Built-in error checking
- Comprehensive final verification

## 🔧 Troubleshooting

### **"Calibration failed" Messages**
- **Action**: Type `CONTINUE` to retry
- **Alternative**: Type `SKIP` to continue anyway
- **Check**: Connections, power supply, stability

### **Unstable Readings**
- **Wait longer** for readings to stabilize
- **Check** for vibrations or air currents
- **Ensure** masses are properly centered

### **Wrong Mass Placement**
- **System guides** you to correct positions
- **Use measuring tape** to verify positions
- **Place masses** as close to load cells as possible

### **Need to Stop Mid-Calibration**
- **Type**: `ABORT` to safely cancel
- **Resume**: Start over with `AUTOMATED_CALIBRATION`
- **Data**: Previous steps may need to be repeated

## 🎉 After Calibration

### **Immediate Next Steps**
```
CAL_FORCE_DATA         # Test calibrated measurements
VAL_START              # Run validation tests  
START                  # Begin data acquisition
```

### **Documentation**
- Record calibration date and conditions
- Document masses and equipment used
- Note any issues or special conditions
- Schedule next calibration (annually)

## 🆚 Manual vs Automated

| Feature | Manual Commands | Automated System |
|---------|----------------|------------------|
| **Complexity** | High - many commands | Low - just CONTINUE |
| **Error Prone** | Easy to make mistakes | Guided process |
| **Time** | Faster if expert | Slightly longer but safer |
| **Documentation** | Manual tracking | Automatic progress |
| **Recovery** | Manual restart | Built-in retry/skip |
| **Learning** | Steep learning curve | Intuitive interface |

## 🎯 Best Practices

### **Before Starting**
- ✅ Read this guide completely
- ✅ Gather all required masses
- ✅ Ensure stable, level surface
- ✅ Allow system warm-up (>10 minutes)

### **During Calibration**
- ✅ Follow instructions exactly
- ✅ Wait for readings to stabilize
- ✅ Place masses carefully and centrally
- ✅ Use `STATUS` if unsure of progress

### **After Calibration**
- ✅ Run validation tests
- ✅ Document the calibration
- ✅ Test with known masses
- ✅ Save calibration certificates

---

## 🚀 Ready to Start?

Simply type:
```
AUTOMATED_CALIBRATION
```

The system will guide you through everything else! 🎉
