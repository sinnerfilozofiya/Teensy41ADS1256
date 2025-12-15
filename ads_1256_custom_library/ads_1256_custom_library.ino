#include <SPI.h>
#include <math.h>

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

// Serial3 pins for ESP32 command interface
// RX3 = Pin 15 (from ESP32 GPIO2) - Used for receiving commands
// TX is on Pin 16 (software serial) - PCB fixed wiring
#define SOFT_TX_PIN 16

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

// Forward declarations for functions in other tabs
void initADS();
void SendCMD(uint8_t cmd);
int32_t read_single_channel_fast(uint8_t channel);

// ADS1256 constants (in case they're not loaded yet from constants tab)
#ifndef SELFCAL
#define SELFCAL 0xF0  // Self Offset Calibration
#endif

// Ensure critical constants are available (Arduino IDE compilation order issue)
#ifndef PGA_64
#define PGA_64 0x27  // B00100111 - PGA = 64
#endif

#ifndef DR_30000
#define DR_30000 0xF0  // B11110000 - 30kSPS
#endif

// Forward declarations for noise filtering functions
struct FilteredReading {
  int32_t raw;
  int32_t filtered;
  bool outlier_detected;
};

// Filter types and structures (needed for Arduino IDE compilation order)
enum FilterType {
  FILTER_NONE = 0,
  FILTER_SMA = 1,        // Simple Moving Average
  FILTER_EMA = 2,        // Exponential Moving Average
  FILTER_MEDIAN = 3,     // Median Filter
  FILTER_KALMAN = 4,     // Kalman Filter
  FILTER_ADAPTIVE = 5,   // Adaptive Filter
  FILTER_COMBINED = 6,   // Combined multi-stage filter
  FILTER_GAUSSIAN = 7    // Gaussian Filter
};

enum OutlierMethod {
  OUTLIER_NONE = 0,
  OUTLIER_ZSCORE = 1,    // Z-score based
  OUTLIER_IQR = 2,       // Interquartile Range
  OUTLIER_MAD = 3,       // Median Absolute Deviation
  OUTLIER_ADAPTIVE = 4   // Adaptive threshold
};

struct FilterConfig {
  FilterType filter_type;
  OutlierMethod outlier_method;
  uint8_t window_size;           // Filter window size
  double outlier_threshold;      // Outlier detection threshold
  double ema_alpha;              // EMA smoothing factor (0-1)
  double gaussian_sigma;         // Gaussian filter standard deviation
  bool enable_preprocessing;     // Enable preprocessing stage
  bool enable_postprocessing;    // Enable postprocessing stage
};

// Circular buffer template
template<typename T, size_t N>
class CircularBuffer {
private:
  T buffer[N];
  size_t head = 0;
  size_t count = 0;
  
public:
  void push(T value) {
    buffer[head] = value;
    head = (head + 1) % N;
    if (count < N) count++;
  }
  
  T get(size_t index) const {
    if (index >= count) return T(0);
    size_t pos = (head - count + index) % N;
    return buffer[pos];
  }
  
  size_t size() const { return count; }
  bool full() const { return count == N; }
  
  T newest() const {
    if (count == 0) return T(0);
    return buffer[(head - 1 + N) % N];
  }
  
  T oldest() const {
    if (count == 0) return T(0);
    return buffer[(head - count + N) % N];
  }
  
  void clear() {
    head = 0;
    count = 0;
  }
};

// Filter state for each channel
struct ChannelFilter {
  CircularBuffer<int32_t, 20> raw_buffer;      // Raw data history
  CircularBuffer<double, 20> filtered_buffer;  // Filtered data history
  
  // EMA state
  double ema_value;
  bool ema_initialized;
  
  // Kalman filter state
  double kalman_x;      // State estimate
  double kalman_P;      // Error covariance
  double kalman_Q;      // Process noise
  double kalman_R;      // Measurement noise
  bool kalman_initialized;
  
  // Gaussian filter state
  double gaussian_weights[11];  // Pre-computed Gaussian weights (up to kernel size 11)
  uint8_t gaussian_kernel_size; // Current kernel size
  bool gaussian_initialized;
  
  // Statistics for outlier detection
  double running_mean;
  double running_variance;
  double running_mad;   // Median Absolute Deviation
  uint32_t sample_count;
  
  // Adaptive thresholds
  double adaptive_threshold_high;
  double adaptive_threshold_low;
  double noise_level;
  
  // Constructor
  ChannelFilter() {
    reset();
  }
  
  void reset() {
    raw_buffer.clear();
    filtered_buffer.clear();
    ema_value = 0.0;
    ema_initialized = false;
    kalman_x = 0.0;
    kalman_P = 1.0;
    kalman_Q = 0.01;
    kalman_R = 0.1;
    kalman_initialized = false;
    gaussian_initialized = false;
    running_mean = 0.0;
    running_variance = 0.0;
    running_mad = 0.0;
    sample_count = 0;
    adaptive_threshold_high = 100000.0;
    adaptive_threshold_low = -100000.0;
    noise_level = 0.0;
  }
};

// Forward declarations for noise filtering functions (defined in noise_filtering.ino)
// Note: channel_filters[] is defined in noise_filtering.ino
int32_t get_filtered_load_cell_reading(uint8_t channel);
FilteredReading get_filtered_reading_with_info(uint8_t channel);
void enable_filtering(bool enable);
void reset_filters();
void set_realtime_filtering();
void set_high_quality_filtering();
void set_low_noise_filtering();
void set_gaussian_filtering();
void set_filter_type(int type);
void set_outlier_method(int method);
void set_gaussian_sigma(double sigma);
void show_filter_status();

// ============================================================================
// SOFTWARE SERIAL TX ON PIN 16 (PCB has TX wired to Pin 16, not Pin 14)
// ============================================================================

#define SOFT_TX_BAUD 9600
#define SOFT_TX_BIT_TIME_US (1000000 / SOFT_TX_BAUD)  // ~104 µs per bit (much more reliable)

void softSerialTX_init() {
    pinMode(SOFT_TX_PIN, OUTPUT);
    digitalWriteFast(SOFT_TX_PIN, HIGH);  // Idle high (UART idle state)
}

void softSerialTX_writeByte(uint8_t b) {
    // Disable interrupts for precise timing
    noInterrupts();
    
    // Start bit (LOW)
    digitalWriteFast(SOFT_TX_PIN, LOW);
    delayMicroseconds(SOFT_TX_BIT_TIME_US);
    
    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        digitalWriteFast(SOFT_TX_PIN, (b >> i) & 0x01);
        delayMicroseconds(SOFT_TX_BIT_TIME_US);
    }
    
    // Stop bit (HIGH)
    digitalWriteFast(SOFT_TX_PIN, HIGH);
    delayMicroseconds(SOFT_TX_BIT_TIME_US);
    
    interrupts();
}

void softSerialTX_print(const char* str) {
    while (*str) {
        softSerialTX_writeByte(*str++);
    }
}

void softSerialTX_println(const char* str) {
    softSerialTX_print(str);
    softSerialTX_writeByte('\n');
}

void softSerialTX_printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    softSerialTX_print(buffer);
}

// ============================================================================
// STATE MANAGEMENT AND DATA ACQUISITION CONTROL
// ============================================================================

const char* get_state_name() {
  switch (current_state) {
    case STATE_STOPPED:  return "STOPPED";
    case STATE_STARTING: return "STARTING";
    case STATE_RUNNING:  return "RUNNING";
    case STATE_STOPPING: return "STOPPING";
    case STATE_ERROR:    return "ERROR";
    default:             return "UNKNOWN";
  }
}

void send_status_response(const char* cmd, const char* status) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s:%s", cmd, status);
  softSerialTX_println(buf);
  Serial.printf("[T41] -> ESP32: %s\n", buf);
}

void send_state_update() {
  char buf[64];
  snprintf(buf, sizeof(buf), "STATE:%s", get_state_name());
  softSerialTX_println(buf);
  Serial.printf("[T41] State update: %s\n", get_state_name());
  state_changed = false;
}

bool start_data_acquisition() {
  if (current_state == STATE_RUNNING) {
    Serial.println("[T41] Already running");
    return true;
  }
  
  Serial.println("[T41] Starting data acquisition...");
  current_state = STATE_STARTING;
  state_changed = true;
  
  // Reset statistics
  st.start_ms = millis();
  st.last_ms = st.start_ms;
  st.frames_sent = 0;
  st.min_xfer_us = UINT32_MAX;
  st.max_xfer_us = 0;
  
  // Reset buffer
  buffer_index = 0;
  frame_idx = 0;
  tick = 0;
  
  current_state = STATE_RUNNING;
  state_changed = true;
  
  Serial.println("[T41] Data acquisition started");
  return true;
}

bool stop_data_acquisition() {
  if (current_state == STATE_STOPPED) {
    Serial.println("[T41] Already stopped");
    return true;
  }
  
  Serial.println("[T41] Stopping data acquisition...");
  current_state = STATE_STOPPING;
  state_changed = true;
  
  // Print final stats
  print_statistics();
  
  current_state = STATE_STOPPED;
  state_changed = true;
  
  Serial.println("[T41] Data acquisition stopped");
  return true;
}

bool restart_data_acquisition() {
  Serial.println("[T41] Restarting data acquisition...");
  stop_data_acquisition();
  delay(100);
  return start_data_acquisition();
}

// ============================================================================
// COMMAND INTERFACE FUNCTIONS
// ============================================================================

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
    else if (command == "STATUS") {
      const char* state_names_arr[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
      char buf[128];
      snprintf(buf, sizeof(buf), "STATUS:state=%s,frames=%lu",
               state_names_arr[current_state], (unsigned long)st.frames_sent);
      softSerialTX_println(buf);
      send_status_response("STATUS", "OK");
    }
    // ========== NOISE FILTERING COMMANDS ==========
    else if (command == "FILTER_ENABLE") {
      enable_filtering(true);
      send_status_response("FILTER_ENABLE", "OK");
    }
    else if (command == "FILTER_DISABLE") {
      enable_filtering(false);
      send_status_response("FILTER_DISABLE", "OK");
    }
    else if (command == "FILTER_STATUS") {
      show_filter_status();
      send_status_response("FILTER_STATUS", "OK");
    }
    else if (command == "FILTER_RESET") {
      reset_filters();
      send_status_response("FILTER_RESET", "OK");
    }
    else if (command == "FILTER_REALTIME") {
      set_realtime_filtering();
      send_status_response("FILTER_REALTIME", "OK");
    }
    else if (command == "FILTER_HIGH_QUALITY") {
      set_high_quality_filtering();
      send_status_response("FILTER_HIGH_QUALITY", "OK");
    }
    else if (command == "FILTER_LOW_NOISE") {
      set_low_noise_filtering();
      send_status_response("FILTER_LOW_NOISE", "OK");
    }
    else if (command == "PING") {
      // Simple ping-pong test for communication chain verification
      Serial.println("[T41] PING received, sending PONG via Software TX (Pin 16)...");
      Serial.printf("[T41] Software TX config: Pin=%d, Baud=%d, BitTime=%d us\n", 
                    SOFT_TX_PIN, SOFT_TX_BAUD, SOFT_TX_BIT_TIME_US);
      
      // Verify pin is configured
      pinMode(SOFT_TX_PIN, OUTPUT);
      
      // Send PONG using software serial TX on Pin 16
      softSerialTX_println("PONG");
      
      Serial.println("[T41] PONG sent on Pin 16 at 9600 baud");
    }
    else if (command == "TX_TEST") {
      // Continuous TX test - send data every 100ms for 5 seconds
      Serial.println("[T41] Starting Software TX test on Pin 16 (5 seconds)...");
      char buf[20];
      for (int i = 0; i < 50; i++) {
        sprintf(buf, "TX_TEST_%d", i);
        softSerialTX_println(buf);
        Serial.printf("[T41] Sent: %s\n", buf);
        delay(100);
      }
      Serial.println("[T41] TX test complete");
    }
    else {
      send_status_response(command.c_str(), "UNKNOWN");
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
    else if (command == "STATUS") {
      const char* state_names[] = {"STOPPED", "STARTING", "RUNNING", "STOPPING", "ERROR"};
      Serial.printf("[T41] State: %s | Frames: %lu | Uptime: %lu ms\n", 
                    state_names[current_state], (unsigned long)st.frames_sent, millis());
    }
    // ========== NOISE FILTERING COMMANDS ==========
    else if (command == "FILTER_ENABLE") {
      enable_filtering(true);
      Serial.println("[T41] ✓ Noise filtering enabled");
    }
    else if (command == "FILTER_DISABLE") {
      enable_filtering(false);
      Serial.println("[T41] ✓ Noise filtering disabled");
    }
    else if (command == "FILTER_STATUS") {
      show_filter_status();
    }
    else if (command == "FILTER_RESET") {
      reset_filters();
      Serial.println("[T41] ✓ All filters reset");
    }
    else if (command == "FILTER_REALTIME") {
      set_realtime_filtering();
      Serial.println("[T41] ✓ Real-time filtering preset applied");
    }
    else if (command == "FILTER_HIGH_QUALITY") {
      set_high_quality_filtering();
      Serial.println("[T41] ✓ High-quality filtering preset applied");
    }
    else if (command == "FILTER_LOW_NOISE") {
      set_low_noise_filtering();
      Serial.println("[T41] ✓ Low-noise filtering preset applied");
    }
    else if (command.startsWith("FILTER_TYPE ")) {
      int type = command.substring(12).toInt();
      if (type >= 0 && type <= 7) {
        set_filter_type(type);
        Serial.printf("[T41] ✓ Filter type set to: %d\n", type);
      } else {
        Serial.println("[T41] ✗ Invalid filter type (0-7)");
      }
    }
    else if (command.startsWith("OUTLIER_METHOD ")) {
      int method = command.substring(15).toInt();
      if (method >= 0 && method <= 4) {
        set_outlier_method(method);
        Serial.printf("[T41] ✓ Outlier method set to: %d\n", method);
      } else {
        Serial.println("[T41] ✗ Invalid outlier method (0-4)");
      }
    }
    else if (command.startsWith("GAUSSIAN_SIGMA ")) {
      double sigma = command.substring(15).toFloat();
      if (sigma > 0.0 && sigma <= 5.0) {
        set_gaussian_sigma(sigma);
        Serial.printf("[T41] ✓ Gaussian sigma set to: %.2f\n", sigma);
      } else {
        Serial.println("[T41] ✗ Invalid sigma value (0.1-5.0)");
      }
    }
    else if (command == "FILTER_GAUSSIAN") {
      set_gaussian_filtering();
      Serial.println("[T41] ✓ Gaussian filtering preset applied");
    }
    else if (command == "SHOW_FILTERED") {
      Serial.println("[T41] Raw vs Filtered Load Cell Readings:");
      for (int ch = 0; ch < 4; ch++) {
        FilteredReading reading = get_filtered_reading_with_info(ch);
        Serial.printf("[T41] LC%d: Raw=%ld, Filtered=%ld, Outlier=%s\n",
                      ch + 1, reading.raw, reading.filtered,
                      reading.outlier_detected ? "YES" : "NO");
      }
    }
    else if (command == "PING") {
      Serial.println("[T41] PING received, sending PONG...");
      softSerialTX_println("PONG");
      Serial.println("[T41] PONG sent");
    }
    else if (command == "TX_TEST") {
      Serial.println("[T41] Starting TX test (50 messages)...");
      char buf[20];
      for (int i = 0; i < 50; i++) {
        sprintf(buf, "TX_TEST_%d", i);
        softSerialTX_println(buf);
        Serial.printf("[T41] Sent: %s\n", buf);
        delay(100);
      }
      Serial.println("[T41] TX test complete");
    }
    else if (command == "HELP") {
      Serial.println("[T41] === COMMANDS ===");
      Serial.println("[T41] Data: START, STOP, RESTART, RESET, STATUS");
      Serial.println("[T41] Filter: FILTER_ENABLE/DISABLE/STATUS/RESET");
      Serial.println("[T41] Presets: FILTER_REALTIME/HIGH_QUALITY/LOW_NOISE/GAUSSIAN");
      Serial.println("[T41] Config: FILTER_TYPE <0-7>, OUTLIER_METHOD <0-4>, GAUSSIAN_SIGMA <0.1-5>");
      Serial.println("[T41] Debug: SHOW_FILTERED, PING, TX_TEST");
    }
    else if (command == "") {
      // Empty command, do nothing
    }
    else {
      Serial.printf("[T41] ✗ Unknown command: '%s' (type HELP)\n", command.c_str());
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
  Serial.println("[T41] ADS1256 Force Plate System");
  
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
  
  // Initialize Serial3 for ESP32 command interface (RX only - Pin 15)
  Serial3.begin(9600);  // Lower baud rate for reliable software TX
  Serial.println("[T41] Serial3 RX initialized (Pin 15) at 9600 baud");
  
  // Initialize Software Serial TX on Pin 16 (PCB fixed wiring)
  softSerialTX_init();
  Serial.println("[T41] Software TX initialized (Pin 16)");

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
  
  // Initialize noise filtering system
  set_realtime_filtering();  // Start with real-time preset
  enable_filtering(true);    // Enable filtering by default
  Serial.println("[T41] Noise filtering enabled (Real-time preset)");
  
  Serial.println("[T41] Ready - type HELP for commands");
  
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
      
      // Read current channel with filtering
      int32_t filtered_value = get_filtered_load_cell_reading(current_channel);
      
      // Store in buffer
      sample_buffer[current_channel][buffer_index] = filtered_value;
      
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

  Serial.printf("[T41] t=%.1fs frames=%lu rate=%.1f kbps sps=%.0f (%.0f/ch) xfer[%lu-%lu us]\n",
    secs, (unsigned long)st.frames_sent, kbps, total_sps, sps_per_channel,
    (unsigned long)(st.min_xfer_us == UINT32_MAX ? 0 : st.min_xfer_us),
    (unsigned long)st.max_xfer_us);
    
  // Performance status
  if (sps_per_channel >= 1000.0) {
    Serial.println("[T41] ✓ 1000+ SPS per channel achieved");
  } else if (sps_per_channel >= 800.0) {
    Serial.println("[T41] ~ Near target SPS");
  }
}
