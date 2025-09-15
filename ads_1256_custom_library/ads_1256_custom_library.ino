#include <SPI.h>
#include <Arduino.h>
#include <math.h>

// ADS1256 pin definitions for Teensy 4.1
#define ADS_RST_PIN    8  //ADS1256 reset pin
#define ADS_RDY_PIN    22 //ADS1256 data ready
#define ADS_CS_PIN     10 //ADS1256 chip select

// SPI1 pins for ESP32 communication
#define PIN_SCK1   27
#define PIN_MOSI1  26
#define PIN_MISO1   1
#define PIN_CS1     0

// Batch frame structure for ESP32 communication
#define BATCH_SIZE 100  // 100 samples per batch
#define BATCHES_PER_SECOND 10  // 10 batches per second = 1000 SPS
#define CHANNELS 4
#define TARGET_SPS 1000  // Target 1000 samples per second

// Single sample structure (12 bytes: 4 channels × 3 bytes each)
struct __attribute__((packed)) Sample {
  uint8_t lc1[3];  // 24-bit little-endian
  uint8_t lc2[3];
  uint8_t lc3[3];
  uint8_t lc4[3];
};

// Batch frame structure (1212 bytes total)
struct __attribute__((packed)) BatchFrame {
  uint8_t  sync[2];           // 2 bytes: 0xA5, 0x5A
  uint8_t  plate_id;          // 1 byte
  uint8_t  proto_ver;         // 1 byte
  uint16_t batch_idx;         // 2 bytes: batch number
  uint32_t t0_us;             // 4 bytes: start timestamp
  uint64_t timestamp_us;      // 8 bytes: precise timestamp
  Sample   samples[BATCH_SIZE]; // 1200 bytes: 100 samples × 12 bytes
  uint16_t crc16;             // 2 bytes: CRC checksum
};

#define BATCH_FRAME_BYTES sizeof(BatchFrame)  // Should be 1220 bytes
static BatchFrame batch_tx;

static inline uint16_t crc16_ccitt_false(const uint8_t* d, uint32_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= ((uint16_t)*d++) << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? ((crc<<1) ^ 0x1021) : (crc<<1);
  }
  return crc;
}

// clamp to signed 24-bit and pack little-endian
static inline void pack_int24_le(uint8_t* p, int32_t v) {
  if (v >  0x7FFFFF) v =  0x7FFFFF;
  if (v < -0x800000) v = -0x800000;
  uint32_t u = (uint32_t)(v & 0xFFFFFF);
  p[0] = (uint8_t)(u & 0xFF);
  p[1] = (uint8_t)((u >> 8) & 0xFF);
  p[2] = (uint8_t)((u >> 16) & 0xFF);
}

struct Stats {
  uint32_t start_ms=0, last_ms=0;
  uint32_t frames_sent=0;
  uint32_t min_xfer_us=UINT32_MAX, max_xfer_us=0;
} st;

uint16_t batch_idx = 0;
elapsedMillis batch_timer;   // Timer for batch transmission (100ms intervals)
uint32_t batch_interval_ms = 100;    // 100 milliseconds = 10 batches per second

//put the ADC constants here
double resolution = 8388608.; //2^23-1
//this needs to match the setting in the ADC init function in the library tab
double Gain = 64.; //be sure to have a period 
double vRef = 5.0; //reference voltage
//we'll calculate this in setup
double bitToVolt = 0.;

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("booting");
  
  // Initialize ADS1256 pins
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);
  
  // Initialize SPI for ADS1256
  SPI.begin();
  
  // Initialize SPI1 for ESP32 communication
  pinMode(PIN_CS1, OUTPUT);
  digitalWriteFast(PIN_CS1, HIGH);
  
  SPI1.setSCK(PIN_SCK1);
  SPI1.setMOSI(PIN_MOSI1);
  SPI1.setMISO(PIN_MISO1);
  SPI1.begin();

  initADS();
  Serial.println("ADS1256 init done");
  Serial.println("SPI1 master for ESP32 ready");

  //determine the conversion factor
  bitToVolt = resolution*Gain/vRef;
  
  // Initialize stats
  st.start_ms = millis();
  st.last_ms = st.start_ms;
}

int32_t val1;  // Load cell 1 (AIN0-AIN1)
int32_t val2;  // Load cell 2 (AIN2-AIN3)
int32_t val3;  // Load cell 3 (AIN4-AIN5)
int32_t val4;  // Load cell 4 (AIN6-AIN7)

void loop() {
  // Check if it's time to collect samples for a new batch (every 100ms = 10 batches per second)
  if (batch_timer >= batch_interval_ms) {
    batch_timer = 0;  // Reset batch timer
    
    // Collect 100 samples for this batch
    for (int sample_idx = 0; sample_idx < BATCH_SIZE; sample_idx++) {
      // Read all 4 load cell channels for each sample
      read_four_values();  // This updates val1, val2, val3, val4 with current readings
      
      // Pack current sample into batch
      Sample* current_sample = &batch_tx.samples[sample_idx];
      pack_int24_le(current_sample->lc1, val1);
      pack_int24_le(current_sample->lc2, val2);
      pack_int24_le(current_sample->lc3, val3);
      pack_int24_le(current_sample->lc4, val4);
      
      // Small delay to achieve ~1000 SPS (1ms per sample)
      delayMicroseconds(1000);
    }
    
    // Prepare batch header
    batch_tx.sync[0] = 0xA5;
    batch_tx.sync[1] = 0x5A;
    batch_tx.plate_id = 1;
    batch_tx.proto_ver = 0x02;  // Version 2 for batch protocol
    batch_tx.batch_idx = batch_idx++;
    batch_tx.t0_us = micros();
    batch_tx.timestamp_us = ((uint64_t)millis() * 1000ULL) + (micros() % 1000);
    
    // Calculate CRC for the batch
    batch_tx.crc16 = 0;
    batch_tx.crc16 = crc16_ccitt_false((uint8_t*)&batch_tx, offsetof(BatchFrame, crc16));
    
    // Send batch to ESP32 via SPI1
    SPISettings set(10'000'000, MSBFIRST, SPI_MODE0);
    uint32_t t0 = micros();
    SPI1.beginTransaction(set);
    digitalWriteFast(PIN_CS1, LOW);
    SPI1.transfer((uint8_t*)&batch_tx, BATCH_FRAME_BYTES);
    digitalWriteFast(PIN_CS1, HIGH);
    SPI1.endTransaction();
    uint32_t dur = micros() - t0;
    if (dur < st.min_xfer_us) st.min_xfer_us = dur;
    if (dur > st.max_xfer_us) st.max_xfer_us = dur;
    
    st.frames_sent++;
  }

  // Print stats every 5 seconds (reduce serial output frequency for higher SPS)
  uint32_t now_ms = millis();
  if (now_ms - st.last_ms >= 5000) {
    double secs = (now_ms - st.start_ms) / 1000.0;
    double kbps = (st.frames_sent * BATCH_FRAME_BYTES * 8.0) / secs / 1000.0;
    double batches_per_sec = st.frames_sent / secs;
    double samples_per_sec = batches_per_sec * BATCH_SIZE;  // Each batch contains 100 samples

    Serial.printf("[T41] t=%.1fs batches=%lu rate=%.1f kbit/s bps=%.1f sps=%.1f xfer_us[min=%lu max=%lu] | LC1:%ld LC2:%ld LC3:%ld LC4:%ld\n",
      secs, (unsigned long)st.frames_sent, kbps, batches_per_sec, samples_per_sec,
      (unsigned long)(st.min_xfer_us==UINT32_MAX?0:st.min_xfer_us),
      (unsigned long)st.max_xfer_us, val1, val2, val3, val4);

    st.last_ms = now_ms;
    st.min_xfer_us = UINT32_MAX; st.max_xfer_us = 0; // reset window
  }
}
