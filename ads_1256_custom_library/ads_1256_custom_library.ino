#include <SPI.h>

// Include ADS1256 constants first
// (These are defined in the ads1256_constants.ino tab)

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
// TX3 = Pin 14 (to ESP32 GPIO3)
// RX3 = Pin 15 (from ESP32 GPIO2)

// ============================================================================
// DATA ACQUISITION STATE MANAGEMENT
// ============================================================================

enum AcquisitionState {
  STATE_STOPPED = 0,      // Data acquisition stopped
  STATE_STARTING = 1,     // Initializing/starting up
  STATE_RUNNING = 2,      // Active data acquisition
  STATE_STOPPING = 3,     // Gracefully stopping
  STATE_ERROR = 4         // Error state
};

static AcquisitionState current_state = STATE_STOPPED;
static bool state_changed = true;  // Flag to report state changes

// ============================================================================
// FORWARD DECLARATIONS AND GLOBAL VARIABLES
// ============================================================================

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

// Load cell values
int32_t val1;  // Load cell 1 (AIN0-AIN1)
int32_t val2;  // Load cell 2 (AIN2-AIN3)
int32_t val3;  // Load cell 3 (AIN4-AIN5)
int32_t val4;  // Load cell 4 (AIN6-AIN7)

// ============================================================================
// CALIBRATION AND OFFSET MANAGEMENT
// ============================================================================

// Zero offsets for each load cell (captured during ZERO command)
int32_t zero_offsets[CHANNELS] = {0, 0, 0, 0};
bool offsets_applied = false;

// Calibration state
bool calibration_mode = false;
int32_t calibration_samples[CHANNELS][100];  // Store 100 samples for averaging
uint8_t calibration_sample_count = 0;

// Forward declarations for functions in other tabs
void initADS();
void SendCMD(uint8_t cmd);
int32_t read_single_channel_fast(uint8_t channel);

// ADS1256 constants (in case they're not loaded yet from constants tab)
#ifndef SELFCAL
#define SELFCAL 0xF0  // Self Offset Calibration
#endif

// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================

void zero_load_cells() {
  Serial.println("[T41] 🔄 Zeroing load cells - capturing offsets...");
  
  // Capture 50 samples from each channel for averaging
  int32_t temp_offsets[CHANNELS] = {0, 0, 0, 0};
  const int num_samples = 1500;
  
  for (int sample = 0; sample < num_samples; sample++) {
    for (int ch = 0; ch < CHANNELS; ch++) {
      int32_t raw_value = read_single_channel_fast(ch);
      temp_offsets[ch] += raw_value;
      delay(2);  // Small delay between readings
    }
  }
  
  // Calculate averages and store as offsets
  for (int ch = 0; ch < CHANNELS; ch++) {
    zero_offsets[ch] = temp_offsets[ch] / num_samples;
    Serial.printf("[T41] LC%d offset: %ld\n", ch + 1, zero_offsets[ch]);
  }
  
  offsets_applied = true;
  Serial.println("[T41] ✅ Load cells zeroed successfully!");
}

void start_calibration() {
  Serial.println("[T41] 🔧 Starting calibration mode...");
  calibration_mode = true;
  calibration_sample_count = 0;
  memset(calibration_samples, 0, sizeof(calibration_samples));
  Serial.println("[T41] Apply known weight and wait for calibration to complete");
}

void stop_calibration() {
  if (!calibration_mode) {
    Serial.println("[T41] ⚠️ Not in calibration mode");
    return;
  }
  
  calibration_mode = false;
  
  if (calibration_sample_count < 10) {
    Serial.println("[T41] ❌ Not enough calibration samples collected");
    return;
  }
  
  Serial.println("[T41] 📊 Calibration results:");
  
  // Calculate averages for each channel
  for (int ch = 0; ch < CHANNELS; ch++) {
    int32_t sum = 0;
    for (int i = 0; i < calibration_sample_count; i++) {
      sum += calibration_samples[ch][i];
    }
    int32_t average = sum / calibration_sample_count;
    int32_t zeroed_value = average - zero_offsets[ch];
    
    Serial.printf("[T41] LC%d: Raw=%ld, Zeroed=%ld (%d samples)\n", 
                  ch + 1, average, zeroed_value, calibration_sample_count);
  }
  
  Serial.println("[T41] ✅ Calibration completed!");
}

void show_current_values() {
  Serial.println("[T41] 📊 Current Load Cell Values:");
  Serial.println("[T41] ================================");
  
  for (int ch = 0; ch < CHANNELS; ch++) {
    int32_t raw_value = read_single_channel_fast(ch);
    int32_t zeroed_value = raw_value - zero_offsets[ch];
    double voltage = raw_value / bitToVolt;
    
    Serial.printf("[T41] LC%d: Raw=%8ld | Zeroed=%8ld | Voltage=%7.4fV\n", 
                  ch + 1, raw_value, zeroed_value, voltage);
    delay(5);
  }
  
  Serial.printf("[T41] Offsets Applied: %s\n", offsets_applied ? "YES" : "NO");
  Serial.println("[T41] ================================");
}

void reset_calibration() {
  Serial.println("[T41] 🔄 Resetting calibration data...");
  
  // Reset offsets
  for (int ch = 0; ch < CHANNELS; ch++) {
    zero_offsets[ch] = 0;
  }
  
  offsets_applied = false;
  calibration_mode = false;
  calibration_sample_count = 0;
  
  Serial.println("[T41] ✅ Calibration data reset!");
}

// Apply offsets to raw reading
int32_t apply_offset(int32_t raw_value, int channel) {
  if (offsets_applied && channel >= 0 && channel < CHANNELS) {
    return raw_value - zero_offsets[channel];
  }
  return raw_value;
}

// ============================================================================
// COMMAND INTERFACE FUNCTIONS
// ============================================================================

void send_status_response(const char* command, const char* status) {
  Serial3.printf("RESP:%s:%s\n", command, status);
  Serial3.flush();
}

void send_state_update() {
  const char* state_names[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
  Serial3.printf("STATE:%s\n", state_names[current_state]);
  Serial3.flush();
  state_changed = false;
}

bool start_data_acquisition() {
  if (current_state == STATE_RUNNING) {
    return true; // Already running
  }
  
  current_state = STATE_STARTING;
  state_changed = true;
  
  Serial.println("[T41] Starting data acquisition...");
  
  // Reset frame index and statistics
  frame_idx = 0;
  st.start_ms = millis();
  st.last_ms = st.start_ms;
  st.frames_sent = 0;
  st.min_xfer_us = UINT32_MAX;
  st.max_xfer_us = 0;
  
  // Clear sample buffer
  memset(sample_buffer, 0, sizeof(sample_buffer));
  buffer_index = 0;
  
  // Reset timing
  tick = 0;
  
  current_state = STATE_RUNNING;
  state_changed = true;
  
  Serial.println("[T41] Data acquisition started successfully");
  return true;
}

bool stop_data_acquisition() {
  if (current_state == STATE_STOPPED) {
    return true; // Already stopped
  }
  
  current_state = STATE_STOPPING;
  state_changed = true;
  
  Serial.println("[T41] Stopping data acquisition...");
  
  // Allow current operations to complete gracefully
  delay(50);
  
  current_state = STATE_STOPPED;
  state_changed = true;
  
  Serial.println("[T41] Data acquisition stopped");
  return true;
}

bool restart_data_acquisition() {
  Serial.println("[T41] Restarting data acquisition...");
  
  if (!stop_data_acquisition()) {
    return false;
  }
  
  delay(100); // Brief pause between stop and start
  
  return start_data_acquisition();
}

void handle_esp32_commands() {
  if (Serial3.available()) {
    String command = Serial3.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.printf("[T41] ESP32 Command: %s\n", command.c_str());
    
    if (command == "START") {
      if (start_data_acquisition()) {
        send_status_response("START", "OK");
      } else {
        send_status_response("START", "ERROR");
      }
    }
    else if (command == "STOP") {
      if (stop_data_acquisition()) {
        send_status_response("STOP", "OK");
      } else {
        send_status_response("STOP", "ERROR");
      }
    }
    else if (command == "RESTART") {
      if (restart_data_acquisition()) {
        send_status_response("RESTART", "OK");
      } else {
        send_status_response("RESTART", "ERROR");
      }
    }
    else if (command == "RESET") {
      send_status_response("RESET", "OK");
      delay(500);
      // Software reset
      SCB_AIRCR = 0x05FA0004; // Teensy software reset
    }
    else if (command == "ZERO") {
      zero_load_cells();
      send_status_response("ZERO", "OK");
    }
    else if (command == "ZERO_STATUS") {
      Serial3.printf("ZERO_STATUS:offsets_applied=%s,lc1=%ld,lc2=%ld,lc3=%ld,lc4=%ld\n", 
                     offsets_applied ? "true" : "false",
                     zero_offsets[0], zero_offsets[1], zero_offsets[2], zero_offsets[3]);
      Serial3.flush();
      send_status_response("ZERO_STATUS", "OK");
    }
    else if (command == "ZERO_RESET") {
      reset_calibration();
      send_status_response("ZERO_RESET", "OK");
    }
    else if (command == "STATUS") {
      const char* state_names[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
      Serial3.printf("STATUS:state=%s,frames=%lu,offsets=%s\n", 
                     state_names[current_state], (unsigned long)st.frames_sent,
                     offsets_applied ? "applied" : "none");
      Serial3.flush();
      send_status_response("STATUS", "OK");
    }
    else if (command == "CAL_START") {
      start_calibration();
      send_status_response("CAL_START", "OK");
    }
    else if (command == "CAL_STOP") {
      stop_calibration();
      send_status_response("CAL_STOP", "OK");
    }
    else if (command == "SHOW_VALUES") {
      show_current_values();
      send_status_response("SHOW_VALUES", "OK");
    }
    else if (command == "RESET_CAL") {
      reset_calibration();
      send_status_response("RESET_CAL", "OK");
    }
    else {
      send_status_response(command.c_str(), "ERROR");
    }
  }
}

void handle_serial_monitor_commands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.printf("[T41] Serial Command: %s\n", command.c_str());
    
    if (command == "START") {
      if (start_data_acquisition()) {
        Serial.println("[T41] ✓ Data acquisition started");
      } else {
        Serial.println("[T41] ✗ Failed to start");
      }
    }
    else if (command == "STOP") {
      if (stop_data_acquisition()) {
        Serial.println("[T41] ✓ Data acquisition stopped");
      } else {
        Serial.println("[T41] ✗ Failed to stop");
      }
    }
    else if (command == "RESTART") {
      if (restart_data_acquisition()) {
        Serial.println("[T41] ✓ Data acquisition restarted");
      } else {
        Serial.println("[T41] ✗ Failed to restart");
      }
    }
    else if (command == "RESET") {
      Serial.println("[T41] Resetting in 1 second...");
      delay(1000);
      SCB_AIRCR = 0x05FA0004; // Teensy software reset
    }
    else if (command == "ZERO") {
      zero_load_cells();
    }
    else if (command == "ZERO_STATUS") {
      Serial.printf("[T41] Zero Status: offsets_applied=%s\n", offsets_applied ? "true" : "false");
      for (int i = 0; i < CHANNELS; i++) {
        Serial.printf("[T41] LC%d offset: %ld\n", i + 1, zero_offsets[i]);
      }
    }
    else if (command == "ZERO_RESET") {
      reset_calibration();
    }
    else if (command == "STATUS") {
      const char* state_names[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
      Serial.printf("[T41] === TEENSY STATUS ===\n");
      Serial.printf("[T41] State: %s\n", state_names[current_state]);
      Serial.printf("[T41] Frames sent: %lu\n", (unsigned long)st.frames_sent);
      Serial.printf("[T41] Offsets applied: %s\n", offsets_applied ? "YES" : "NO");
      Serial.printf("[T41] Uptime: %lu ms\n", millis());
      Serial.println("[T41] =======================");
    }
    else if (command == "CAL_START") {
      start_calibration();
    }
    else if (command == "CAL_STOP") {
      stop_calibration();
    }
    else if (command == "SHOW_VALUES" || command == "SHOW") {
      show_current_values();
    }
    else if (command == "RESET_CAL") {
      reset_calibration();
    }
    else if (command == "HELP") {
      Serial.println("[T41] ==========================================");
      Serial.println("[T41] AVAILABLE COMMANDS:");
      Serial.println("[T41] ==========================================");
      Serial.println("[T41] Data Acquisition:");
      Serial.println("[T41]   START       - Start data acquisition");
      Serial.println("[T41]   STOP        - Stop data acquisition");
      Serial.println("[T41]   RESTART     - Restart data acquisition");
      Serial.println("[T41]   RESET       - Reset Teensy");
      Serial.println("[T41] ");
      Serial.println("[T41] Calibration:");
      Serial.println("[T41]   ZERO        - Zero all load cells (capture offsets)");
      Serial.println("[T41]   ZERO_STATUS - Show current zero offsets");
      Serial.println("[T41]   ZERO_RESET  - Reset zero offsets");
      Serial.println("[T41]   CAL_START   - Start calibration mode");
      Serial.println("[T41]   CAL_STOP    - Stop calibration and show results");
      Serial.println("[T41]   SHOW_VALUES - Show current load cell readings");
      Serial.println("[T41]   RESET_CAL   - Reset all calibration data");
      Serial.println("[T41] ");
      Serial.println("[T41] System:");
      Serial.println("[T41]   STATUS      - Show system status");
      Serial.println("[T41] ");
      Serial.println("[T41]   HELP        - Show this help");
      Serial.println("[T41] ==========================================");
    }
    else if (command == "") {
      // Empty command, do nothing
    }
    else {
      Serial.printf("[T41] ✗ Unknown command: '%s'\n", command.c_str());
      Serial.println("[T41] Type 'HELP' for available commands");
    }
  }
}


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
  
  // Initialize in STOPPED state - wait for ESP32 commands
  current_state = STATE_STOPPED;
  state_changed = true;
  
  //determine the conversion factor
  bitToVolt = resolution*Gain/vRef;
  
  // Initialize statistics
  st.start_ms = millis();
  st.last_ms = st.start_ms;
  
  Serial.println("[T41] Ready - waiting for commands");
  Serial.println("[T41] Type 'HELP' for available commands");
  Serial.println("[T41] Quick start: ZERO (to zero), START (to begin)");
  Serial.println("[T41] ==========================================");
  
  // Send initial state to ESP32
  send_state_update();
}

void loop() {
  // Handle ESP32 commands first
  handle_esp32_commands();
  
  // Handle Serial Monitor commands for testing
  handle_serial_monitor_commands();
  
  // Send state updates when state changes
  if (state_changed) {
    send_state_update();
  }
  
  // Only perform data acquisition if in RUNNING state
  if (current_state == STATE_RUNNING) {
    // Sample at 1kHz per channel (4kHz total) - rotate through channels every 250μs
    static elapsedMicros sample_timer;
    static uint8_t current_channel = 0;
    static uint32_t sample_count = 0;
    
    if (sample_timer >= 250) {  // 4kHz total sampling rate
      sample_timer = 0;
      
      // Read current channel only
      int32_t raw_value = read_single_channel_fast(current_channel);
      
      // Apply offset if enabled
      int32_t processed_value = apply_offset(raw_value, current_channel);
      
      // Store in appropriate buffer
      sample_buffer[current_channel][buffer_index] = processed_value;
      
      // Collect calibration samples if in calibration mode
      if (calibration_mode && calibration_sample_count < 100) {
        calibration_samples[current_channel][calibration_sample_count] = raw_value;
        
        // Only increment counter when all channels have been sampled
        if (current_channel == CHANNELS - 1) {
          calibration_sample_count++;
          if (calibration_sample_count >= 100) {
            Serial.println("[T41] 📊 Calibration samples collected (100 samples per channel)");
            Serial.println("[T41] Use 'CAL_STOP' to finish calibration");
          }
        }
      }
      
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

    // Print statistics every 5 seconds (only when running)
    uint32_t now_ms = millis();
    if (now_ms - st.last_ms >= 5000) {
      print_statistics();
      st.last_ms = now_ms;
      st.min_xfer_us = UINT32_MAX; 
      st.max_xfer_us = 0; // reset window
    }
  } else {
    // When not running, just handle commands and small delay
    delay(10);
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
