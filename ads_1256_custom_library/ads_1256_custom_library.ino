#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//(other stuff for getting the ADS1526 to work is in the next tab
// Updated pin definitions for Teensy 4.1
#define ADS_RST_PIN    8  //ADS1256 reset pin (same as Teensy 3.1)
#define ADS_RDY_PIN    22 //ADS1256 data ready (same as Teensy 3.1) 
#define ADS_CS_PIN     10 //ADS1256 chip select (Teensy 4.1 default SPI CS)

/* 
    Teensy 4.1 SPI0 pins:
    CLK  - pin 13 (SCK)
    DIN  - pin 11 (MOSI)
    DOUT - pin 12 (MISO)
    CS   - pin 10 (default CS, but we define our own above)
*/


//put the ADC constants here

double resolution = 8388608.; //2^23-1

//this needs to match the setting in the ADC init function in the library tab
double Gain = 64.; //be sure to have a period 

double vRef = 5.0; //reference voltage

//we'll calculate this in setup
double bitToVolt = 0.;

// Statistics tracking variables
unsigned long statsStartTime = 0;
unsigned long sampleCount = 0;
float samplesPerSecond = 0.0;
unsigned long loopStartTime = 0;
unsigned long loopTime = 0;  // Current loop time (global for access in functions)
unsigned long totalLoopTime = 0;
float avgLoopTime = 0.0;
unsigned long statsInterval = 5000; // Collect stats for 5 seconds
unsigned long displayUpdateInterval = 1000; // Update display every 1 second
bool statsValid = false; // Flag to indicate if we have valid statistics

// Per-channel statistics
unsigned long channelSampleCount[4] = {0, 0, 0, 0}; // Track samples for each channel
float channelSPS[4] = {0.0, 0.0, 0.0, 0.0}; // SPS for each channel

// Data optimization settings
enum OutputMode {
  VERBOSE_MODE,    // Full text output (current mode)
  COMPACT_MODE,    // Minimal text output
  BINARY_MODE,     // Binary output (most efficient)
  DELTA_MODE,      // Delta compression + binary
  ULTRA_MODE       // Ultra-compressed mode (2 bytes per channel)
};

OutputMode currentMode = COMPACT_MODE; // Default to compact mode
bool enableStatsPrint = false; // Disable stats printing for wireless mode
uint8_t decimationFactor = 1; // Send every Nth sample (1 = all samples)
unsigned long decimationCounter = 0;

// Delta compression variables
int32_t lastVal1 = 0, lastVal2 = 0, lastVal3 = 0, lastVal4 = 0;
bool firstSample = true;

// Configuration functions
void setWirelessMode() {
  currentMode = ULTRA_MODE;     // Ultra compression (8 bytes per sample)
  enableStatsPrint = false;     // Disable verbose stats
  decimationFactor = 2;         // Send every 2nd sample (50% reduction)
  Serial.println("# Wireless mode: Ultra compression + 2x decimation");
}

void setUltraMode() {
  currentMode = ULTRA_MODE;     // Ultra compression only
  enableStatsPrint = false;     // Disable verbose stats
  decimationFactor = 1;         // Send all samples
  Serial.println("# Ultra mode: 8 bytes per sample");
}

void setCompactMode() {
  currentMode = COMPACT_MODE;   // CSV format
  enableStatsPrint = false;     // Disable verbose stats
  decimationFactor = 1;         // Send all samples
  Serial.println("# Compact mode: CSV format");
}

void setVerboseMode() {
  currentMode = VERBOSE_MODE;   // Full text output
  enableStatsPrint = true;      // Enable all stats
  decimationFactor = 1;         // Send all samples
  Serial.println("# Verbose mode: Full text output");
}

void setBinaryMode() {
  currentMode = BINARY_MODE;    // Binary format
  enableStatsPrint = false;     // Disable verbose stats
  decimationFactor = 1;         // Send all samples
  Serial.println("# Binary mode: 18 bytes per sample");
}

void setup() {
  delay(1000);
  // Teensy 4.1 supports higher baud rates than 3.1
  Serial.begin(115200);
  Serial.println("booting");
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ADS1256 Monitor"));
  display.println(F("Initializing..."));
  display.display();
  
  //initialize the ADS
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);

  SPI.begin();

  initADS();
  Serial.println("done init");

  //determine the conversion factor
    //do some calculations for the constants
  bitToVolt = resolution*Gain/vRef;
  
  // Initialize statistics
  statsStartTime = millis();
  
  // Set to ultra-compressed slave mode for maximum efficiency
  setUltraMode();               // 8 bytes per sample (33% smaller than binary)
  
  Serial.println("ULTRA_FAST_READY"); // Single startup message
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("ADS1256 Ready"));
  display.display();
  delay(1000);
}

int32_t val1;  // Load cell 1 (AIN0-AIN1)
int32_t val2;  // Load cell 2 (AIN2-AIN3)
int32_t val3;  // Load cell 3 (AIN4-AIN5)
int32_t val4;  // Load cell 4 (AIN6-AIN7)

// Optimized data output functions
void sendCompactData() {
  // Compact CSV format: val1,val2,val3,val4
  Serial.print(val1);
  Serial.print(",");
  Serial.print(val2);
  Serial.print(",");
  Serial.print(val3);
  Serial.print(",");
  Serial.println(val4);
}

void sendBinaryData() {
  // Ultra-compact binary format: 12 bytes total (3 bytes per channel)
  // ADS1256 is 24-bit, so we only need 3 bytes per channel
  // No markers needed for slave mode - master handles framing
  
  // Channel 1 (3 bytes, little endian)
  Serial.write((uint8_t)(val1 & 0xFF));
  Serial.write((uint8_t)((val1 >> 8) & 0xFF));
  Serial.write((uint8_t)((val1 >> 16) & 0xFF));
  
  // Channel 2 (3 bytes, little endian)
  Serial.write((uint8_t)(val2 & 0xFF));
  Serial.write((uint8_t)((val2 >> 8) & 0xFF));
  Serial.write((uint8_t)((val2 >> 16) & 0xFF));
  
  // Channel 3 (3 bytes, little endian)
  Serial.write((uint8_t)(val3 & 0xFF));
  Serial.write((uint8_t)((val3 >> 8) & 0xFF));
  Serial.write((uint8_t)((val3 >> 16) & 0xFF));
  
  // Channel 4 (3 bytes, little endian)
  Serial.write((uint8_t)(val4 & 0xFF));
  Serial.write((uint8_t)((val4 >> 8) & 0xFF));
  Serial.write((uint8_t)((val4 >> 16) & 0xFF));
}

void sendUltraCompressedData() {
  // ULTRA MODE: 8 bytes total (2 bytes per channel)
  // Optimized for maximum speed - minimal processing overhead
  
  // Direct bit manipulation for speed (shift right 8 bits)
  // Send directly without intermediate variables to save time
  
  // Channel 1 - send upper 16 bits directly
  Serial.write((uint8_t)(val1 >> 8));   // Low byte of compressed value
  Serial.write((uint8_t)(val1 >> 16));  // High byte of compressed value
  
  // Channel 2
  Serial.write((uint8_t)(val2 >> 8));
  Serial.write((uint8_t)(val2 >> 16));
  
  // Channel 3  
  Serial.write((uint8_t)(val3 >> 8));
  Serial.write((uint8_t)(val3 >> 16));
  
  // Channel 4
  Serial.write((uint8_t)(val4 >> 8));
  Serial.write((uint8_t)(val4 >> 16));
}

void sendDeltaCompressedData() {
  // Delta compression: only send changes from last reading
  if (firstSample) {
    // Send full values for first sample
    Serial.write(0xFF); // Full sample marker
    sendBinaryData();
    lastVal1 = val1; lastVal2 = val2; lastVal3 = val3; lastVal4 = val4;
    firstSample = false;
    return;
  }
  
  // Calculate deltas
  int32_t delta1 = val1 - lastVal1;
  int32_t delta2 = val2 - lastVal2;
  int32_t delta3 = val3 - lastVal3;
  int32_t delta4 = val4 - lastVal4;
  
  // Check if deltas fit in 16 bits (-32768 to 32767)
  if (abs(delta1) < 32768 && abs(delta2) < 32768 && 
      abs(delta3) < 32768 && abs(delta4) < 32768) {
    // Send compressed deltas (2 bytes per channel = 8 bytes total)
    Serial.write(0xDD); // Delta marker
    Serial.write((uint8_t)(delta1 & 0xFF));
    Serial.write((uint8_t)((delta1 >> 8) & 0xFF));
    Serial.write((uint8_t)(delta2 & 0xFF));
    Serial.write((uint8_t)((delta2 >> 8) & 0xFF));
    Serial.write((uint8_t)(delta3 & 0xFF));
    Serial.write((uint8_t)((delta3 >> 8) & 0xFF));
    Serial.write((uint8_t)(delta4 & 0xFF));
    Serial.write((uint8_t)((delta4 >> 8) & 0xFF));
  } else {
    // Delta too large, send full values
    Serial.write(0xFF); // Full sample marker
    sendBinaryData();
  }
  
  // Update last values
  lastVal1 = val1; lastVal2 = val2; lastVal3 = val3; lastVal4 = val4;
}

void sendVerboseData() {
  // Original verbose format
  Serial.print("LC1: ");
  Serial.print(val1);
  Serial.print("\tLC2: ");
  Serial.print(val2);
  Serial.print("\tLC3: ");
  Serial.print(val3);
  Serial.print("\tLC4: ");
  Serial.print(val4);
  
  if (enableStatsPrint) {
    Serial.print("\tSamples: ");
    Serial.print(sampleCount);
    if (statsValid) {
      Serial.print("\tTotal SPS: ");
      Serial.print(samplesPerSecond, 1);
      Serial.print("\tCH SPS: ");
      Serial.print(channelSPS[0], 0);
      Serial.print("/");
      Serial.print(channelSPS[1], 0);
      Serial.print("/");
      Serial.print(channelSPS[2], 0);
      Serial.print("/");
      Serial.print(channelSPS[3], 0);
    }
    Serial.print("\tLoop: ");
    Serial.print(loopTime / 1000.0, 2);
    Serial.println(" ms");
  } else {
    Serial.println();
  }
}

// Simplified display for slave mode
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.setCursor(0, 0);
  display.println(F("ADS1256 SLAVE"));
  display.drawLine(0, 8, 128, 8, SSD1306_WHITE);
  
  // Current sample count
  display.setCursor(0, 12);
  display.print(F("Samples: "));
  display.println(sampleCount);
  
  // Data rate (8 bytes per sample in ultra mode)
  unsigned long elapsed = millis() - statsStartTime;
  if (elapsed > 1000) {
    float sps = (sampleCount * 1000.0) / elapsed;
    float dataRate = sps * 8; // 8 bytes per sample in ultra mode
    
    display.setCursor(0, 22);
    display.print(F("SPS: "));
    display.print(sps, 1);
    
    display.setCursor(0, 32);
    display.print(F("Data: "));
    if (dataRate > 1000) {
      display.print(dataRate / 1000.0, 1);
      display.println(F(" KB/s"));
    } else {
      display.print(dataRate, 0);
      display.println(F(" B/s"));
    }
  }
  
  // Current values (last reading)
  display.setCursor(0, 42);
  display.print(F("LC1:"));
  display.print(val1);
  
  display.setCursor(0, 52);
  display.print(F("LC2:"));
  display.print(val2);
  
  display.display();
}

// Function to calculate and update statistics
void updateStatistics() {
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - statsStartTime;
  
  // Check if we've collected data for 5 seconds
  if (elapsed >= statsInterval) {
    // Calculate overall samples per second over the 5-second period
    samplesPerSecond = (sampleCount * 1000.0) / elapsed;
    
    // Calculate per-channel SPS
    for (int i = 0; i < 4; i++) {
      channelSPS[i] = (channelSampleCount[i] * 1000.0) / elapsed;
    }
    
    // Calculate average loop time
    if (sampleCount > 0) {
      avgLoopTime = totalLoopTime / (float)sampleCount;
    }
    
    // Mark statistics as valid (after first calculation)
    statsValid = true;
    
    // Reset for next 5-second collection period
    statsStartTime = currentTime;
    sampleCount = 0;
    totalLoopTime = 0;
    
    // Reset per-channel counters
    for (int i = 0; i < 4; i++) {
      channelSampleCount[i] = 0;
    }
    
    // Print updated statistics to serial
    Serial.println("=== 5-Second Statistics Update ===");
    Serial.print("Total SPS: ");
    Serial.println(samplesPerSecond, 1);
    Serial.print("Channel SPS - CH1: ");
    Serial.print(channelSPS[0], 1);
    Serial.print(", CH2: ");
    Serial.print(channelSPS[1], 1);
    Serial.print(", CH3: ");
    Serial.print(channelSPS[2], 1);
    Serial.print(", CH4: ");
    Serial.println(channelSPS[3], 1);
    Serial.print("Average Loop Time: ");
    Serial.print(avgLoopTime / 1000.0, 2);
    Serial.println(" ms");
    Serial.println("==================================");
  }
}

void loop() {
  // ULTRA-FAST SLAVE MODE: Maximum throughput priority
  
  // Read all 4 load cells
  read_four_values();
  
  // Send ultra-compressed data immediately (8 bytes per sample)
  sendUltraCompressedData();
  
  // Minimal statistics (only increment counter)
  sampleCount++;
  
  // Update display every 5 seconds (minimize overhead)
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 5000) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

//
//double value = 0;
//for (int i = 0; i <10; i++){
//  value += read_Value();
//}
//
//value /= 10.;
//value /= bitToVolt;
//
//Serial.println(value,11);
}
