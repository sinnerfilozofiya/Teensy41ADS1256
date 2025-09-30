#include <SPI.h>

// ADS1256 pin definitions (SPI0)
#define ADS_RST_PIN    8  //ADS1256 reset pin (same as Teensy 3.1)
#define ADS_RDY_PIN    22 //ADS1256 data ready (same as Teensy 3.1) 
#define ADS_CS_PIN     10 //ADS1256 chip select (Teensy 4.1 default SPI CS)

/* 
    Teensy 4.1 SPI0 pins (for ADS1256):
    CLK  - pin 13 (SCK)
    DIN  - pin 11 (MOSI)
    DOUT - pin 12 (MISO)
    CS   - pin 10 (default CS, but we define our own above)
*/

// SPI1 pins for ESP32 communication
#define PIN_SCK1   27
#define PIN_MOSI1  26
#define PIN_MISO1   1
#define PIN_CS1     0

// Serial3 pins for ESP32 command interface (Hardware pins - no defines needed)
// TX3 = Pin 34 (to ESP32 GPIO3)
// RX3 = Pin 35 (from ESP32 GPIO2)

// Frame structure constants
#define FRAME_BYTES 144
#define SAMPLES_PER_FRAME 10
#define CHANNELS 4
#define FS_HZ 4000.0  // 4kHz total (1kHz per channel)

// Performance mode selection
#define USE_CONTINUOUS_MODE 0  // Set to 1 for experimental continuous mode, 0 for optimized standard mode

struct __attribute__((packed)) Frame {
  uint8_t  sync[2];
  uint8_t  plate_id, proto_ver;
  uint16_t frame_idx;
  uint32_t t0_us;
  uint8_t  samples[120]; // 10 × (4 ch × int24)
  uint16_t crc16;
  uint8_t  pad[12];
};
static Frame tx;


//put the ADC constants here
double resolution = 8388608.; //2^23-1
//this needs to match the setting in the ADC init function in the library tab
double Gain = 64.; //be sure to have a period 
double vRef = 5.0; //reference voltage
//we'll calculate this in setup
double bitToVolt = 0.;

// CRC16 function (CCITT-FALSE)
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

// Statistics structure
struct Stats {
  uint32_t start_ms=0, last_ms=0;
  uint32_t frames_sent=0;
  uint32_t min_xfer_us=UINT32_MAX, max_xfer_us=0;
} st;

// Sample buffer for 4 channels
int32_t sample_buffer[CHANNELS][SAMPLES_PER_FRAME];
uint8_t buffer_index = 0;
elapsedMillis tick;
uint16_t frame_idx = 0;

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("[T41] Booting ADS1256 + SPI1 Master");
  
  // Initialize ADS1256 (SPI0)
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_RDY_PIN, INPUT);
  pinMode(ADS_RST_PIN, OUTPUT);
  SPI.begin();
  initADS();
  Serial.println("[T41] ADS1256 initialized");

  // Initialize SPI1 for ESP32 communication
  pinMode(PIN_CS1, OUTPUT);
  digitalWriteFast(PIN_CS1, HIGH);
  SPI1.setSCK(PIN_SCK1);
  SPI1.setMOSI(PIN_MOSI1);
  SPI1.setMISO(PIN_MISO1);
  SPI1.begin();
  Serial.println("[T41] SPI1 Master initialized");
  
  // Initialize Serial3 for ESP32 command interface
  Serial3.begin(115200);
  Serial.println("[T41] Serial3 initialized for ESP32 commands (TX=Pin34, RX=Pin35)");

  // Initialize sample buffer
  memset(sample_buffer, 0, sizeof(sample_buffer));
  
  //determine the conversion factor
  bitToVolt = resolution*Gain/vRef;
  
  // Initialize statistics
  st.start_ms = millis();
  st.last_ms = st.start_ms;
  
  Serial.println("[T41] Ready - starting 1kHz per channel sampling (4kSPS total) with 10ms frame transmission");
}

int32_t val1;  // Load cell 1 (AIN0-AIN1)
int32_t val2;  // Load cell 2 (AIN2-AIN3)
int32_t val3;  // Load cell 3 (AIN4-AIN5)
int32_t val4;  // Load cell 4 (AIN6-AIN7)

void loop() {
  // Sample at 1kHz per channel (4kHz total) - rotate through channels every 250μs
  static elapsedMicros sample_timer;
  static uint8_t current_channel = 0;
  static uint32_t sample_count = 0;
  
  if (sample_timer >= 250) {  // 4kHz total sampling rate
    sample_timer = 0;
    
    // Read current channel only
    int32_t current_value = read_single_channel_fast(current_channel);
    
    // Store in appropriate buffer
    sample_buffer[current_channel][buffer_index] = current_value;
    
    // Move to next channel
    current_channel++;
    if (current_channel >= CHANNELS) {
      current_channel = 0;
      // All 4 channels sampled, increment buffer index
      buffer_index++;
      if (buffer_index >= SAMPLES_PER_FRAME) {
        buffer_index = 0;
      }
    }
    
    sample_count++;
  }

  // Send frame every 10ms (100 Hz frame rate) for stable transmission
  if (tick >= 10) {
    uint32_t actual_interval = tick;
    tick = 0;
    send_frame_to_esp32();
    
    // Warn if timing is off by more than 1ms
    if (actual_interval > 11 || actual_interval < 9) {
      Serial.printf("[T41] WARNING: Frame interval %lums (expected 10ms)\n", actual_interval);
    }
  }

  // Print statistics every 5 seconds
  uint32_t now_ms = millis();
  if (now_ms - st.last_ms >= 5000) {
    print_statistics();
    st.last_ms = now_ms;
    st.min_xfer_us = UINT32_MAX; 
    st.max_xfer_us = 0; // reset window
  }
}

void send_frame_to_esp32() {
  // Fill frame header
  tx.sync[0] = 0xA5; 
  tx.sync[1] = 0x5A;
  tx.plate_id = 1; 
  tx.proto_ver = 0x01;
  tx.frame_idx = frame_idx++;
  tx.t0_us = micros();

  // Pack 10 samples from each of the 4 channels into the frame
  uint8_t* s = tx.samples;
  for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
    // Get the sample index (most recent samples first)
    int sample_idx = (buffer_index - SAMPLES_PER_FRAME + i + SAMPLES_PER_FRAME) % SAMPLES_PER_FRAME;
    
    // Pack 4 channels × 3 bytes each (24-bit little-endian)
    pack_int24_le(s + 0,  sample_buffer[0][sample_idx]); // LC1
    pack_int24_le(s + 3,  sample_buffer[1][sample_idx]); // LC2
    pack_int24_le(s + 6,  sample_buffer[2][sample_idx]); // LC3
    pack_int24_le(s + 9,  sample_buffer[3][sample_idx]); // LC4
    s += (CHANNELS * 3); // 4ch × 3 bytes = 12 bytes per sample
  }

  // Clear padding
  memset(tx.pad, 0, sizeof(tx.pad));

  // Calculate CRC16 over header + samples (bytes 0..129)
  tx.crc16 = 0;
  tx.crc16 = crc16_ccitt_false((uint8_t*)&tx, offsetof(Frame, crc16));

  // SPI transfer to ESP32 at 10 MHz
  SPISettings settings(10000000, MSBFIRST, SPI_MODE0);
  uint32_t t0 = micros();
  SPI1.beginTransaction(settings);
  digitalWriteFast(PIN_CS1, LOW);
  SPI1.transfer((uint8_t*)&tx, FRAME_BYTES);
  digitalWriteFast(PIN_CS1, HIGH);
  SPI1.endTransaction();
  uint32_t dur = micros() - t0;
  
  // Update statistics
  if (dur < st.min_xfer_us) st.min_xfer_us = dur;
  if (dur > st.max_xfer_us) st.max_xfer_us = dur;
  st.frames_sent++;
}

void print_statistics() {
  double secs = (millis() - st.start_ms) / 1000.0;
  double kbps = (st.frames_sent * FRAME_BYTES * 8.0) / secs / 1000.0;
  double total_sps = (st.frames_sent * SAMPLES_PER_FRAME * CHANNELS) / secs; // 4 channels per sample
  double sps_per_channel = total_sps / CHANNELS;

  Serial.printf("[T41] t=%.1fs sent=%lu rate=%.1f kbit/s samples=%.1f sps (%.1f per ch) xfer_us[min=%lu max=%lu] ROTATING\n",
    secs, (unsigned long)st.frames_sent, kbps, total_sps, sps_per_channel,
    (unsigned long)(st.min_xfer_us == UINT32_MAX ? 0 : st.min_xfer_us),
    (unsigned long)st.max_xfer_us);
    
  // Performance analysis
  if (sps_per_channel >= 1000.0) {
    Serial.println("[T41] ✓ TARGET ACHIEVED: 1000+ SPS per channel!");
  } else if (sps_per_channel >= 800.0) {
    Serial.println("[T41] ⚠ CLOSE: Near target, consider further optimization");
  } else {
    Serial.println("[T41] ✗ BELOW TARGET: Performance optimization needed");
  }
}
