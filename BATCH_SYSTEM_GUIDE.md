# ESP32 ADS1256 Batch Data Collection System

## 🎯 System Overview

This is a robust data collection system that:
- **Teensy 4.1**: Reads 4 load cells via ADS1256 ADC at 1000 SPS
- **ESP32-S3**: Receives batch data via SPI and outputs to serial
- **Python**: Collects and saves all individual samples to CSV

## 📊 Data Flow

```
ADS1256 → Teensy 4.1 → ESP32-S3 → Python → CSV File
(4 Load Cells) (Batch Creator) (Serial Output) (Data Collector) (All Samples)
```

## 🔧 System Architecture

### **Teensy 4.1 (Master)**
- **Sampling**: Reads all 4 load cells every 1ms (1000 SPS total)
- **Batching**: Collects 100 complete samples (all 4 channels) per batch
- **Transmission**: Sends 1 batch every 100ms (10 batches/second)
- **Data**: Each sample contains real ADS1256 readings from all 4 load cells

### **ESP32-S3 (Slave)**  
- **Reception**: Receives 1220-byte batch frames via SPI
- **Processing**: Unpacks all 100 samples from each batch
- **Output**: Sends all individual samples to serial for Python collection

### **Python Collector**
- **Parsing**: Captures all individual samples between batch markers
- **Storage**: Saves every sample to CSV with timestamp
- **Monitoring**: Real-time progress and statistics

## 📈 Performance Specifications

| Metric | Value |
|--------|-------|
| **Sample Rate** | 1000 SPS (samples per second) |
| **Batch Rate** | 10 BPS (batches per second) |
| **Samples per Batch** | 100 samples |
| **Channels per Sample** | 4 load cells |
| **Data per Batch** | 1220 bytes |
| **Total Data Rate** | ~98 kbit/s |

## 🚀 Usage Instructions

### **1. Upload Teensy Code**
```cpp
// File: ads_1256_custom_library/ads_1256_custom_library.ino
// - Reads ADS1256 every 1ms
// - Creates batches of 100 samples
// - Sends via SPI1 to ESP32
```

### **2. Upload ESP32 Code**
```cpp
// File: esp32_ads1256_slave/esp32_ads1256_slave.ino  
// - Receives SPI batch data
// - Outputs all samples to serial
// - Provides batch markers and stats
```

### **3. Run Python Collector**
```bash
python esp32_batch_collector.py COM10 10
```
- Collects for 10 seconds
- Saves all samples to timestamped CSV file
- Shows real-time progress

## 📊 Expected Output

### **ESP32 Serial Output:**
```
[ESP] Batch:42 | Samples:100 | TotalSamples:4200 | diff:1250 us
[ESP] BatchData:42 Start
[ESP] Sample:42:0:12345678:23456789:34567890:45678901
[ESP] Sample:42:1:12345679:23456790:34567891:45678902
...
[ESP] Sample:42:99:12345777:23456888:34567999:45679000
[ESP] BatchData:42 End
```

### **Python Console Output:**
```
📦 BATCH 42 START - collecting samples...
📊 Batch 42: 100 samples, Total: 4200
💾 Sample B:42 S: 0 | LC1:12345678 LC2:23456789 LC3:34567890 LC4:45678901
💾 Sample B:42 S:25 | LC1:12345703 LC2:23456814 LC3:34567915 LC4:45679026
✅ BATCH 42 END - saved 100 samples

📈 PROGRESS: 5.0s elapsed | Batches: 50 (10.0/s) | Samples: 5000 (1000 SPS) | Remaining: 5.0s
```

### **CSV File Structure:**
```csv
timestamp,batch_id,sample_index,lc1,lc2,lc3,lc4
1694789123.45,42,0,12345678,23456789,34567890,45678901
1694789123.45,42,1,12345679,23456790,34567891,45678902
1694789123.45,42,2,12345680,23456791,34567892,45678903
...
```

## 🔍 Key Features

### **Robust Data Collection**
- ✅ All 4 load cells read simultaneously for each sample
- ✅ Real ADS1256 sensor data (not test data)
- ✅ Complete batch integrity with CRC checking
- ✅ Every individual sample saved to CSV

### **High Performance**
- ✅ True 1000 SPS data collection
- ✅ Efficient batch transmission (10x fewer SPI transfers)
- ✅ Non-blocking data processing
- ✅ Real-time monitoring and statistics

### **Data Integrity**
- ✅ Batch markers ensure complete data capture
- ✅ CRC16 error checking on all transmissions
- ✅ Timestamp synchronization analysis
- ✅ Missing batch detection

## 🛠️ Troubleshooting

### **Low Sample Rate**
- Check ADS1256 DRDY signal
- Verify SPI connections
- Monitor ESP32 serial output rate

### **Missing Samples**
- Increase Python serial buffer
- Check for CRC errors in ESP32 stats
- Verify batch start/end markers

### **Connection Issues**
- Verify COM port in Python script
- Check ESP32 power and connections
- Monitor Teensy serial output

## 📋 File Structure

```
Teensy41ADS1256/
├── ads_1256_custom_library/
│   ├── ads_1256_custom_library.ino    # Main Teensy code
│   ├── ads_1256_stuff.ino             # ADS1256 functions
│   └── ads1256_constants.ino          # Constants
├── esp32_ads1256_slave/
│   └── esp32_ads1256_slave.ino        # ESP32 slave code
├── esp32_batch_collector.py           # Python data collector
└── BATCH_SYSTEM_GUIDE.md             # This guide
```

## 🎉 Expected Results

After running the system for 10 seconds, you should get:
- **~100 batches** received and processed
- **~10,000 individual samples** saved to CSV
- **Complete load cell data** from all 4 channels
- **Real-time performance monitoring**

The CSV file will contain every single ADS1256 reading with precise timestamps, ready for analysis in Excel or any data analysis tool.
