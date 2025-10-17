# Interactive Automated Calibration Interface Guide

## Overview
The ESP32 SPI slave now provides a complete interactive interface for automated calibration of the Teensy force plate system. You can control and monitor the entire calibration process directly from the ESP32 serial terminal, with real-time display of all `[AUTO-CAL]` messages from the Teensy.

## 🚀 Quick Start

### 1. Connect and Power On
1. Connect ESP32 to Teensy via UART (GPIO2→Pin15, GPIO3→Pin14)
2. Power on both systems
3. Open ESP32 Serial Monitor at 921600 baud

### 2. Start Automated Calibration
```
AUTOMATED_CALIBRATION
```
You'll see the full calibration introduction and prompts.

### 3. Follow Interactive Prompts
```
CONTINUE    # Proceed to next step
SKIP        # Skip current step
ABORT       # Cancel calibration
STATUS      # Show current progress
```

## 📋 Complete Command Reference

### **Automated Calibration Commands**

#### **Start Calibration**
- `AUTOMATED_CALIBRATION` - Start the full automated calibration process
- `AUTO_CAL_START` - Alternative start command

#### **Interactive Control**
- `CONTINUE` - Proceed to next calibration step
- `SKIP` - Skip current calibration step
- `ABORT` - Cancel automated calibration
- `STATUS` - Show current calibration progress

#### **Configuration Commands**
- `CAL_CONFIG_SHOW` - Display current calibration settings
- `CAL_CONFIG_MASSES 5,10,15,20` - Set custom calibration masses
- `CAL_CONFIG_RESET_MASSES` - Reset to default masses (0,5,10,15,20 kg)
- `CAL_CONFIG_ENABLE_STEP_C` - Enable matrix calibration (Step C)
- `CAL_CONFIG_DISABLE_STEP_C` - Disable matrix calibration
- `CAL_CONFIG_CENTER_PLACEMENT` - Use center mass placement
- `CAL_CONFIG_CORNER_PLACEMENT` - Use individual corner placement

### **Calibration Commands**
- `ZERO` - Zero all load cells (1500 samples)
- `ZERO_STATUS` - Show zeroing status and offsets
- `ZERO_RESET` - Reset zero offsets
- `CAL_SUMMARY` - Show calibration summary
- `CAL_FORCE_DATA` - Get calibrated force data
- `CAL_FORCE_INT16` - Get force data as int16 values
- `SHOW_VALUES` - Show current load cell values
- `RESET_CAL` - Reset all calibration data

### **Validation & Diagnostics**
- `VAL_CALIBRATION` - Validate calibration accuracy
- `VAL_FORCE_ACCURACY` - Test force measurement accuracy
- `VAL_COP_ACCURACY` - Test center of pressure accuracy
- `DIAG_ADC` - Run ADC diagnostics
- `DIAG_LOAD_CELLS` - Run load cell diagnostics
- `DIAG_CALIBRATION` - Run calibration diagnostics

### **Noise Filtering**
- `FILTER_OFF` - Disable all filtering
- `FILTER_LOW_PASS` - Enable low pass filter
- `FILTER_MOVING_AVG` - Enable moving average filter
- `FILTER_MEDIAN` - Enable median filter
- `FILTER_GAUSSIAN` - Enable Gaussian filter
- `FILTER_TYPE 0-7` - Set specific filter type
- `OUTLIER_METHOD 0-4` - Set outlier detection method
- `GAUSSIAN_SIGMA 0.1-5.0` - Set Gaussian filter sigma

### **System Commands**
- `STATUS` - Show complete system status
- `AUTO_CAL_MONITOR_ON/OFF` - Enable/disable real-time monitoring
- `DEBUG_ON/OFF` - Enable/disable raw data output
- `TEST_CALIBRATION` - Test calibration commands
- `HELP` - Show complete command list

## 🎯 Interactive Calibration Workflow

### **Step 1: Preparation**
```
CAL_CONFIG_SHOW          # Check current settings
CAL_CONFIG_MASSES 5,10,15,20  # Set your masses
CAL_CONFIG_CENTER_PLACEMENT   # Choose placement method
```

### **Step 2: Start Calibration**
```
AUTOMATED_CALIBRATION    # Start the process
```
You'll see:
```
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 🚀 AUTOMATED FORCE PLATE CALIBRATION
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] This automated system will guide you through
[SPI_SLAVE] [AUTO-CAL] the flexible calibration process:
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] 📋 STEP A: ADC (ADS1256) Calibration
[SPI_SLAVE] [AUTO-CAL] 📋 STEP B: Load Cell Calibration (CENTER placement)
[SPI_SLAVE] [AUTO-CAL] 📋 STEP C: Matrix Calibration (OPTIONAL)
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] Ready to begin? Type 'CONTINUE' to start...
```

### **Step 3: Interactive Control**
```
CONTINUE    # Proceed to ADC calibration
```
You'll see real-time progress:
```
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 📋 STEP A: ADC (ADS1256) CALIBRATION
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] 🎯 PURPOSE:
[SPI_SLAVE] [AUTO-CAL]   Calibrate the ADS1256 ADC for accurate voltage
[SPI_SLAVE] [AUTO-CAL]   measurements at your current PGA and data rate
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] ⏱️  DURATION: ~30 seconds
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] Ready to start ADC calibration?
[SPI_SLAVE] [AUTO-CAL] Type 'CONTINUE' to proceed...
```

### **Step 4: Monitor Progress**
All calibration steps will show detailed progress:
```
[SPI_SLAVE] [AUTO-CAL] 🔧 Starting ADC calibration...
[SPI_SLAVE] [AUTO-CAL] 📊 SELFOCAL: Offset calibration
[SPI_SLAVE] [AUTO-CAL] 📊 SELFGCAL: Gain calibration
[SPI_SLAVE] [AUTO-CAL] ✅ ADC calibration complete!
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] Moving to Step B: Load Cell Calibration...
```

## 🔧 Advanced Features

### **Real-time Monitoring**
- All `[AUTO-CAL]` messages are displayed in real-time
- No message loss during calibration process
- Continuous monitoring without blocking other operations

### **Interactive Control**
- Respond to prompts directly from ESP32 terminal
- Skip steps if needed
- Abort calibration at any time
- Check status anytime

### **Configuration Management**
- Set custom calibration masses
- Choose center vs corner placement
- Enable/disable matrix calibration
- View current configuration

### **Comprehensive Testing**
- Test all calibration commands
- Validate calibration accuracy
- Run diagnostics on all components
- Monitor system performance

## 📊 Example Calibration Session

```
[SPI_SLAVE] ESP32 SPI Slave ready for commands
[SPI_SLAVE] Type 'HELP' for complete command list
[SPI_SLAVE] ==========================================

> CAL_CONFIG_SHOW
[SPI_SLAVE] Processing calibration command: CAL_CONFIG_SHOW
[SPI_SLAVE] Sending command: CAL_CONFIG_SHOW
[SPI_SLAVE] Response Summary:
  Command: CAL_CONFIG_SHOW
  Type: OK
  Status: OK
  Success: YES
  Response Time: 45 ms
[SPI_SLAVE] ✓ Calibration command 'CAL_CONFIG_SHOW' executed successfully

> AUTOMATED_CALIBRATION
[SPI_SLAVE] Processing calibration command: AUTOMATED_CALIBRATION
[SPI_SLAVE] Sending command: AUTOMATED_CALIBRATION
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 🚀 AUTOMATED FORCE PLATE CALIBRATION
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] This automated system will guide you through
[SPI_SLAVE] [AUTO-CAL] the flexible calibration process:
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] 📋 STEP A: ADC (ADS1256) Calibration
[SPI_SLAVE] [AUTO-CAL] 📋 STEP B: Load Cell Calibration (CENTER placement)
[SPI_SLAVE] [AUTO-CAL] 📋 STEP C: Matrix Calibration (OPTIONAL)
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] Ready to begin? Type 'CONTINUE' to start...

> CONTINUE
[SPI_SLAVE] Processing automated calibration command: CONTINUE
[SPI_SLAVE] Sent 'CONTINUE' to Teensy for automated calibration
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 📋 STEP A: ADC (ADS1256) CALIBRATION
[SPI_SLAVE] [AUTO-CAL] ==========================================
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] 🎯 PURPOSE:
[SPI_SLAVE] [AUTO-CAL]   Calibrate the ADS1256 ADC for accurate voltage
[SPI_SLAVE] [AUTO-CAL]   measurements at your current PGA and data rate
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] ⏱️  DURATION: ~30 seconds
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] Ready to start ADC calibration?
[SPI_SLAVE] [AUTO-CAL] Type 'CONTINUE' to proceed...

> CONTINUE
[SPI_SLAVE] Processing automated calibration command: CONTINUE
[SPI_SLAVE] Sent 'CONTINUE' to Teensy for automated calibration
[SPI_SLAVE] [AUTO-CAL] 🔧 Starting ADC calibration...
[SPI_SLAVE] [AUTO-CAL] 📊 SELFOCAL: Offset calibration
[SPI_SLAVE] [AUTO-CAL] 📊 SELFGCAL: Gain calibration
[SPI_SLAVE] [AUTO-CAL] ✅ ADC calibration complete!
[SPI_SLAVE] [AUTO-CAL] 
[SPI_SLAVE] [AUTO-CAL] Moving to Step B: Load Cell Calibration...
```

## 🛠️ Troubleshooting

### **No Messages Appearing**
1. Check if monitoring is enabled: `STATUS`
2. Enable monitoring: `AUTO_CAL_MONITOR_ON`
3. Verify UART connection: `TEST_UART`

### **Commands Not Working**
1. Check Teensy connection: `PING`
2. Test UART: `TEST_UART`
3. Check command format: `HELP`

### **Calibration Issues**
1. Check system status: `STATUS`
2. Run diagnostics: `DIAG_CALIBRATION`
3. Validate calibration: `VAL_CALIBRATION`

## 🎉 Benefits

1. **Complete Control**: Full calibration control from ESP32 terminal
2. **Real-time Monitoring**: See all calibration progress in real-time
3. **Interactive Interface**: Respond to prompts directly
4. **Comprehensive Commands**: Access to all calibration features
5. **Easy Configuration**: Simple commands to change settings
6. **Robust Testing**: Built-in validation and diagnostics
7. **No Message Loss**: Continuous monitoring ensures no missed messages

This interactive interface provides complete control over the automated calibration process while maintaining full visibility into the Teensy's calibration progress through real-time message monitoring.
