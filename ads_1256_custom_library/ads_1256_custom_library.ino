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

// Ensure critical constants are available (Arduino IDE compilation order issue)
#ifndef PGA_64
#define PGA_64 0x27  // B00100111 - PGA = 64
#endif

#ifndef DR_30000
#define DR_30000 0xF0  // B11110000 - 30kSPS
#endif

// ForceData structure definition (needed for compilation - MUST come before function declarations)
#ifndef FORCE_DATA_STRUCT_DEFINED
#define FORCE_DATA_STRUCT_DEFINED
struct ForceData {
  double fz;        // Vertical force (N)
  double mx;        // Moment about X axis (N⋅m)
  double my;        // Moment about Y axis (N⋅m)
  double cop_x;     // Center of pressure X (mm)
  double cop_y;     // Center of pressure Y (mm)
  bool valid;       // Data validity
};
#endif

// Forward declarations for calibration functions
bool perform_adc_self_calibration(uint8_t pga_setting, uint8_t data_rate);
bool restore_adc_calibration();
bool start_load_cell_tare(uint8_t channel);
bool start_load_cell_span_calibration(uint8_t channel);
bool add_span_calibration_point(uint8_t channel, double mass_kg);
bool compute_span_calibration(uint8_t channel);
bool perform_shunt_calibration(uint8_t channel);
bool start_matrix_calibration();
bool add_matrix_calibration_point(double mass_kg, double x_mm, double y_mm);
bool compute_matrix_calibration();
bool save_calibration_to_eeprom();
bool load_calibration_from_eeprom();
void clear_calibration_data();
void print_calibration_summary();
ForceData get_calibrated_force_data();

// Forward declarations for validation functions
bool start_accuracy_validation();
bool add_validation_test(const char* name, double mass_kg, double x_mm, double y_mm);
bool run_validation_tests();
void show_validation_results();
bool run_repeatability_test(double mass_kg, double x_mm, double y_mm, int num_trials);
void run_load_cell_diagnostics();
void run_adc_diagnostics();

// Forward declarations for automated calibration functions
void start_automated_calibration();
void handle_auto_cal_input(String command);
void update_automated_calibration();
bool is_auto_calibration_active();
void handle_auto_cal_continue();

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
    kalman_Q = 0.1;
    kalman_R = 1.0;
    kalman_initialized = false;
    gaussian_kernel_size = 0;
    gaussian_initialized = false;
    memset(gaussian_weights, 0, sizeof(gaussian_weights));
    running_mean = 0.0;
    running_variance = 0.0;
    running_mad = 0.0;
    sample_count = 0;
    adaptive_threshold_high = 5000.0;
    adaptive_threshold_low = -5000.0;
    noise_level = 100.0;
  }
};

double apply_noise_filter(int32_t raw_value, uint8_t channel);
int32_t get_filtered_load_cell_reading(uint8_t channel);
void set_filter_type(int type);
void set_outlier_method(int method);
void set_gaussian_sigma(double sigma);
void set_gaussian_filtering();
void enable_filtering(bool enable);
void reset_filters();
void show_filter_status();
void set_realtime_filtering();
void set_high_quality_filtering();
void set_low_noise_filtering();
FilteredReading get_filtered_reading_with_info(uint8_t channel);

// Force data conversion functions
int16_t force_to_int16_decigrams(double force_newtons);
int16_t moment_to_int16_scaled(double moment_nm);
int16_t cop_to_int16_mm(double cop_mm);

// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================

void zero_load_cells() {
  Serial.println("[T41] 🔄 Zeroing load cells - capturing offsets...");
  Serial.printf("[T41] 📊 Collecting %d samples from 4 channels (estimated time: ~12 seconds)\n", 1500);
  
  // Capture 1500 samples from each channel for averaging
  int32_t temp_offsets[CHANNELS] = {0, 0, 0, 0};
  const int num_samples = 1500;
  
  for (int sample = 0; sample < num_samples; sample++) {
    for (int ch = 0; ch < CHANNELS; ch++) {
      int32_t raw_value = read_single_channel_fast(ch);
      temp_offsets[ch] += raw_value;
      delay(2);  // Small delay between readings
    }
    
    // Progress updates every 300 samples (every ~2.4 seconds)
    if ((sample + 1) % 300 == 0) {
      float progress = ((float)(sample + 1) / num_samples) * 100.0;
      Serial.printf("[T41] 📈 Progress: %.1f%% (%d/%d samples)\n", progress, sample + 1, num_samples);
    }
  }
  
  Serial.println("[T41] 🧮 Calculating averages...");
  
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
    FilteredReading reading = get_filtered_reading_with_info(ch);
    int32_t zeroed_filtered = reading.filtered - zero_offsets[ch];
    double voltage = reading.raw / bitToVolt;
    
    Serial.printf("[T41] LC%d: Raw=%8ld | Filtered=%8ld | Zeroed=%8ld | Voltage=%7.4fV %s\n", 
                  ch + 1, reading.raw, reading.filtered, zeroed_filtered, voltage,
                  reading.outlier_detected ? "[OUTLIER]" : "");
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

// ============================================================================
// DUAL OUTPUT SYSTEM FOR ESP32 MESSAGE RELAY
// ============================================================================

// Dual output functions to send messages to both Serial (USB) and Serial3 (ESP32)
void dual_print(const String& message) {
  Serial.print(message);
  Serial3.print(message);
}

void dual_println(const String& message) {
  Serial.println(message);
  Serial3.println(message);
}

void dual_printf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.print(buffer);
  Serial3.print(buffer);
}

// Auto calibration specific dual output (with AUTO-CAL prefix for ESP32 parsing)
void auto_cal_println(const String& message) {
  Serial.println(message);
  Serial3.println(message);
  Serial3.flush(); // Ensure immediate transmission to ESP32
}

void auto_cal_printf(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  Serial.print(buffer);
  Serial3.print(buffer);
  Serial3.flush(); // Ensure immediate transmission to ESP32
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
    // ========== COMPREHENSIVE CALIBRATION COMMANDS ==========
    else if (command == "CAL_LOAD") {
      if (load_calibration_from_eeprom()) {
        send_status_response("CAL_LOAD", "OK");
      } else {
        send_status_response("CAL_LOAD", "ERROR");
      }
    }
    else if (command == "CAL_SAVE") {
      if (save_calibration_to_eeprom()) {
        send_status_response("CAL_SAVE", "OK");
      } else {
        send_status_response("CAL_SAVE", "ERROR");
      }
    }
    else if (command == "CAL_CLEAR") {
      clear_calibration_data();
      send_status_response("CAL_CLEAR", "OK");
    }
    else if (command == "CAL_SUMMARY") {
      print_calibration_summary();
      send_status_response("CAL_SUMMARY", "OK");
    }
    else if (command == "CAL_ADC") {
      if (perform_adc_self_calibration(PGA_64, DR_30000)) {
        send_status_response("CAL_ADC", "OK");
      } else {
        send_status_response("CAL_ADC", "ERROR");
      }
    }
    else if (command.startsWith("CAL_TARE_")) {
      uint8_t channel = command.charAt(9) - '1'; // CAL_TARE_1, CAL_TARE_2, etc.
      if (channel < 4 && start_load_cell_tare(channel)) {
        send_status_response(command.c_str(), "OK");
      } else {
        send_status_response(command.c_str(), "ERROR");
      }
    }
    else if (command.startsWith("CAL_SPAN_")) {
      uint8_t channel = command.charAt(9) - '1';
      if (channel < 4 && start_load_cell_span_calibration(channel)) {
        send_status_response(command.c_str(), "OK");
      } else {
        send_status_response(command.c_str(), "ERROR");
      }
    }
    else if (command == "CAL_MATRIX_START") {
      if (start_matrix_calibration()) {
        send_status_response("CAL_MATRIX_START", "OK");
      } else {
        send_status_response("CAL_MATRIX_START", "ERROR");
      }
    }
    else if (command == "CAL_MATRIX_COMPUTE") {
      if (compute_matrix_calibration()) {
        send_status_response("CAL_MATRIX_COMPUTE", "OK");
      } else {
        send_status_response("CAL_MATRIX_COMPUTE", "ERROR");
      }
    }
    else if (command == "CAL_FORCE_DATA") {
      ForceData data = get_calibrated_force_data();
      if (data.valid) {
        // Convert to int16 format with 10g precision
        int16_t fz_10g = force_to_int16_decigrams(data.fz);
        int16_t mx_scaled = moment_to_int16_scaled(data.mx);
        int16_t my_scaled = moment_to_int16_scaled(data.my);
        int16_t cop_x_mm = cop_to_int16_mm(data.cop_x);
        int16_t cop_y_mm = cop_to_int16_mm(data.cop_y);
        
        Serial3.printf("FORCE_DATA:%d,%d,%d,%d,%d\n",
                       fz_10g, mx_scaled, my_scaled, cop_x_mm, cop_y_mm);
        send_status_response("CAL_FORCE_DATA", "OK");
      } else {
        send_status_response("CAL_FORCE_DATA", "ERROR");
      }
    }
    else if (command == "CAL_FORCE_INT16") {
      ForceData data = get_calibrated_force_data();
      if (data.valid) {
        // Convert to int16 format with 10g precision
        int16_t fz_10g = force_to_int16_decigrams(data.fz);
        int16_t mx_scaled = moment_to_int16_scaled(data.mx);
        int16_t my_scaled = moment_to_int16_scaled(data.my);
        int16_t cop_x_mm = cop_to_int16_mm(data.cop_x);
        int16_t cop_y_mm = cop_to_int16_mm(data.cop_y);
        
        Serial3.printf("FORCE_INT16:%d,%d,%d,%d,%d\n",
                       fz_10g, mx_scaled, my_scaled, cop_x_mm, cop_y_mm);
        send_status_response("CAL_FORCE_INT16", "OK");
      } else {
        send_status_response("CAL_FORCE_INT16", "ERROR");
      }
    }
    // ========== VALIDATION AND DIAGNOSTIC COMMANDS ==========
    else if (command == "VAL_START") {
      if (start_accuracy_validation()) {
        send_status_response("VAL_START", "OK");
      } else {
        send_status_response("VAL_START", "ERROR");
      }
    }
    else if (command == "VAL_RUN_TESTS") {
      if (run_validation_tests()) {
        send_status_response("VAL_RUN_TESTS", "OK");
      } else {
        send_status_response("VAL_RUN_TESTS", "ERROR");
      }
    }
    else if (command == "VAL_SHOW_RESULTS") {
      show_validation_results();
      send_status_response("VAL_SHOW_RESULTS", "OK");
    }
    else if (command == "DIAG_LC") {
      run_load_cell_diagnostics();
      send_status_response("DIAG_LC", "OK");
    }
    else if (command == "DIAG_ADC") {
      run_adc_diagnostics();
      send_status_response("DIAG_ADC", "OK");
    }
    // ========== AUTOMATED CALIBRATION COMMANDS ==========
    else if (command == "AUTO_CAL_START" || command == "AUTOMATED_CALIBRATION") {
      start_automated_calibration();
      send_status_response("AUTO_CAL_START", "OK");
    }
    else if (command == "CONTINUE" || command == "SKIP" || command == "ABORT") {
      if (is_auto_calibration_active()) {
        handle_auto_cal_input(command);
        send_status_response(command.c_str(), "OK");
      } else {
        send_status_response(command.c_str(), "ERROR");
      }
    }
    else if (command == "AUTO_CAL_STATUS") {
      if (is_auto_calibration_active()) {
        show_auto_cal_status();
        send_status_response("AUTO_CAL_STATUS", "OK");
      } else {
        send_status_response("AUTO_CAL_STATUS", "ERROR");
      }
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
    // ========== COMPREHENSIVE CALIBRATION COMMANDS ==========
    else if (command == "CAL_LOAD") {
      if (load_calibration_from_eeprom()) {
        Serial.println("[T41] ✓ Calibration data loaded from EEPROM");
      } else {
        Serial.println("[T41] ✗ Failed to load calibration data");
      }
    }
    else if (command == "CAL_SAVE") {
      if (save_calibration_to_eeprom()) {
        Serial.println("[T41] ✓ Calibration data saved to EEPROM");
      } else {
        Serial.println("[T41] ✗ Failed to save calibration data");
      }
    }
    else if (command == "CAL_CLEAR") {
      clear_calibration_data();
      Serial.println("[T41] ✓ All calibration data cleared");
    }
    else if (command == "CAL_SUMMARY") {
      print_calibration_summary();
    }
    else if (command == "CAL_ADC") {
      Serial.println("[T41] Starting ADC self-calibration...");
      if (perform_adc_self_calibration(PGA_64, DR_30000)) {
        Serial.println("[T41] ✓ ADC calibration completed");
      } else {
        Serial.println("[T41] ✗ ADC calibration failed");
      }
    }
    else if (command.startsWith("CAL_TARE_")) {
      uint8_t channel = command.charAt(9) - '1';
      if (channel < 4) {
        if (start_load_cell_tare(channel)) {
          Serial.printf("[T41] ✓ Load cell %d tare completed\n", channel + 1);
        } else {
          Serial.printf("[T41] ✗ Load cell %d tare failed\n", channel + 1);
        }
      } else {
        Serial.println("[T41] ✗ Invalid channel (use CAL_TARE_1 to CAL_TARE_4)");
      }
    }
    else if (command.startsWith("CAL_SPAN_")) {
      uint8_t channel = command.charAt(9) - '1';
      if (channel < 4) {
        if (start_load_cell_span_calibration(channel)) {
          Serial.printf("[T41] ✓ Started span calibration for load cell %d\n", channel + 1);
        } else {
          Serial.printf("[T41] ✗ Failed to start span calibration for load cell %d\n", channel + 1);
        }
      } else {
        Serial.println("[T41] ✗ Invalid channel (use CAL_SPAN_1 to CAL_SPAN_4)");
      }
    }
    else if (command.startsWith("CAL_SPAN_ADD ")) {
      // Parse: CAL_SPAN_ADD <channel> <mass_kg>
      int space1 = command.indexOf(' ', 13);
      if (space1 > 0) {
        uint8_t channel = command.substring(13, space1).toInt() - 1;
        double mass = command.substring(space1 + 1).toFloat();
        if (channel < 4 && mass > 0) {
          if (add_span_calibration_point(channel, mass)) {
            Serial.printf("[T41] ✓ Added span point: LC%d = %.3f kg\n", channel + 1, mass);
          } else {
            Serial.println("[T41] ✗ Failed to add span point");
          }
        } else {
          Serial.println("[T41] ✗ Invalid parameters (use: CAL_SPAN_ADD <1-4> <mass_kg>)");
        }
      }
    }
    else if (command.startsWith("CAL_SPAN_COMPUTE ")) {
      uint8_t channel = command.substring(17).toInt() - 1;
      if (channel < 4) {
        if (compute_span_calibration(channel)) {
          Serial.printf("[T41] ✓ Span calibration computed for LC%d\n", channel + 1);
        } else {
          Serial.printf("[T41] ✗ Failed to compute span calibration for LC%d\n", channel + 1);
        }
      } else {
        Serial.println("[T41] ✗ Invalid channel (use: CAL_SPAN_COMPUTE <1-4>)");
      }
    }
    else if (command.startsWith("CAL_SHUNT_")) {
      uint8_t channel = command.charAt(10) - '1';
      if (channel < 4) {
        if (perform_shunt_calibration(channel)) {
          Serial.printf("[T41] ✓ Shunt calibration completed for LC%d\n", channel + 1);
        } else {
          Serial.printf("[T41] ✗ Shunt calibration failed for LC%d\n", channel + 1);
        }
      } else {
        Serial.println("[T41] ✗ Invalid channel (use CAL_SHUNT_1 to CAL_SHUNT_4)");
      }
    }
    else if (command == "CAL_MATRIX_START") {
      if (start_matrix_calibration()) {
        Serial.println("[T41] ✓ Matrix calibration started");
      } else {
        Serial.println("[T41] ✗ Failed to start matrix calibration");
      }
    }
    else if (command.startsWith("CAL_MATRIX_ADD ")) {
      // Parse: CAL_MATRIX_ADD <mass_kg> <x_mm> <y_mm>
      int space1 = command.indexOf(' ', 15);
      int space2 = command.indexOf(' ', space1 + 1);
      if (space1 > 0 && space2 > 0) {
        double mass = command.substring(15, space1).toFloat();
        double x = command.substring(space1 + 1, space2).toFloat();
        double y = command.substring(space2 + 1).toFloat();
        if (add_matrix_calibration_point(mass, x, y)) {
          Serial.printf("[T41] ✓ Added matrix point: %.3f kg at (%.1f, %.1f) mm\n", mass, x, y);
        } else {
          Serial.println("[T41] ✗ Failed to add matrix point");
        }
      } else {
        Serial.println("[T41] ✗ Invalid format (use: CAL_MATRIX_ADD <mass_kg> <x_mm> <y_mm>)");
      }
    }
    else if (command == "CAL_MATRIX_COMPUTE") {
      if (compute_matrix_calibration()) {
        Serial.println("[T41] ✓ Matrix calibration computed");
      } else {
        Serial.println("[T41] ✗ Failed to compute matrix calibration");
      }
    }
    else if (command == "CAL_FORCE_DATA") {
      ForceData data = get_calibrated_force_data();
      if (data.valid) {
        // Convert to int16 format with 10g precision
        int16_t fz_10g = force_to_int16_decigrams(data.fz);
        int16_t mx_scaled = moment_to_int16_scaled(data.mx);
        int16_t my_scaled = moment_to_int16_scaled(data.my);
        int16_t cop_x_mm = cop_to_int16_mm(data.cop_x);
        int16_t cop_y_mm = cop_to_int16_mm(data.cop_y);
        
        Serial.printf("[T41] Force Data (int16 format, 10g precision):\n");
        Serial.printf("[T41]   Fz: %d (10g units, max 20000 = 200kg)\n", fz_10g);
        Serial.printf("[T41]   Mx: %d (scaled moment)\n", mx_scaled);
        Serial.printf("[T41]   My: %d (scaled moment)\n", my_scaled);
        Serial.printf("[T41]   COP: (%d, %d) mm\n", cop_x_mm, cop_y_mm);
      } else {
        Serial.println("[T41] ✗ No valid calibrated force data available");
      }
    }
    else if (command == "CAL_FORCE_INT16") {
      ForceData data = get_calibrated_force_data();
      if (data.valid) {
        // Convert to int16 format with 10g precision
        int16_t fz_10g = force_to_int16_decigrams(data.fz);
        int16_t mx_scaled = moment_to_int16_scaled(data.mx);
        int16_t my_scaled = moment_to_int16_scaled(data.my);
        int16_t cop_x_mm = cop_to_int16_mm(data.cop_x);
        int16_t cop_y_mm = cop_to_int16_mm(data.cop_y);
        
        Serial.printf("[T41] %d,%d,%d,%d,%d\n",
                      fz_10g, mx_scaled, my_scaled, cop_x_mm, cop_y_mm);
      } else {
        Serial.println("[T41] ✗ No valid calibrated force data available");
      }
    }
    // ========== VALIDATION AND DIAGNOSTIC COMMANDS ==========
    else if (command == "VAL_START") {
      if (start_accuracy_validation()) {
        Serial.println("[T41] ✓ Validation system started");
      } else {
        Serial.println("[T41] ✗ Failed to start validation");
      }
    }
    else if (command.startsWith("VAL_ADD_TEST ")) {
      // Parse: VAL_ADD_TEST <name> <mass_kg> <x_mm> <y_mm>
      int space1 = command.indexOf(' ', 13);
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);
      
      if (space1 > 0 && space2 > 0 && space3 > 0) {
        String name = command.substring(13, space1);
        double mass = command.substring(space1 + 1, space2).toFloat();
        double x = command.substring(space2 + 1, space3).toFloat();
        double y = command.substring(space3 + 1).toFloat();
        
        if (add_validation_test(name.c_str(), mass, x, y)) {
          Serial.printf("[T41] ✓ Added validation test: %s\n", name.c_str());
        } else {
          Serial.println("[T41] ✗ Failed to add validation test");
        }
      } else {
        Serial.println("[T41] ✗ Invalid format (use: VAL_ADD_TEST <name> <mass_kg> <x_mm> <y_mm>)");
      }
    }
    else if (command == "VAL_RUN_TESTS") {
      if (run_validation_tests()) {
        Serial.println("[T41] ✓ Validation tests completed");
      } else {
        Serial.println("[T41] ✗ Validation tests failed");
      }
    }
    else if (command == "VAL_SHOW_RESULTS") {
      show_validation_results();
    }
    else if (command.startsWith("VAL_REPEATABILITY ")) {
      // Parse: VAL_REPEATABILITY <mass_kg> <x_mm> <y_mm> <trials>
      int space1 = command.indexOf(' ', 19);
      int space2 = command.indexOf(' ', space1 + 1);
      int space3 = command.indexOf(' ', space2 + 1);
      
      if (space1 > 0 && space2 > 0 && space3 > 0) {
        double mass = command.substring(19, space1).toFloat();
        double x = command.substring(space1 + 1, space2).toFloat();
        double y = command.substring(space2 + 1, space3).toFloat();
        int trials = command.substring(space3 + 1).toInt();
        
        if (run_repeatability_test(mass, x, y, trials)) {
          Serial.println("[T41] ✓ Repeatability test passed");
        } else {
          Serial.println("[T41] ✗ Repeatability test failed");
        }
      } else {
        Serial.println("[T41] ✗ Invalid format (use: VAL_REPEATABILITY <mass_kg> <x_mm> <y_mm> <trials>)");
      }
    }
    else if (command == "DIAG_LC") {
      run_load_cell_diagnostics();
    }
    else if (command == "DIAG_ADC") {
      run_adc_diagnostics();
    }
    // ========== AUTOMATED CALIBRATION COMMANDS ==========
    else if (command == "AUTO_CAL_START" || command == "AUTOMATED_CALIBRATION") {
      start_automated_calibration();
    }
    else if (command == "CONTINUE" || command == "SKIP" || command == "ABORT") {
      if (is_auto_calibration_active()) {
        handle_auto_cal_input(command);
      } else {
        Serial.println("[T41] ✗ No automated calibration active");
        Serial.println("[T41] Use 'AUTO_CAL_START' or 'AUTOMATED_CALIBRATION' to begin");
      }
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
      Serial.println("[T41] Basic Calibration:");
      Serial.println("[T41]   ZERO        - Zero all load cells (capture offsets)");
      Serial.println("[T41]   ZERO_STATUS - Show current zero offsets");
      Serial.println("[T41]   ZERO_RESET  - Reset zero offsets");
      Serial.println("[T41]   CAL_START   - Start calibration mode");
      Serial.println("[T41]   CAL_STOP    - Stop calibration and show results");
      Serial.println("[T41]   SHOW_VALUES - Show current load cell readings");
      Serial.println("[T41]   RESET_CAL   - Reset all calibration data");
      Serial.println("[T41] ");
      Serial.println("[T41] Advanced Calibration System:");
      Serial.println("[T41]   CAL_LOAD    - Load calibration from EEPROM");
      Serial.println("[T41]   CAL_SAVE    - Save calibration to EEPROM");
      Serial.println("[T41]   CAL_CLEAR   - Clear all calibration data");
      Serial.println("[T41]   CAL_SUMMARY - Show calibration summary");
      Serial.println("[T41] ");
      Serial.println("[T41] Step A - ADC Calibration:");
      Serial.println("[T41]   CAL_ADC     - Perform ADC self-calibration");
      Serial.println("[T41] ");
      Serial.println("[T41] Step B - Load Cell Calibration:");
      Serial.println("[T41]   CAL_TARE_<1-4>     - Tare individual load cell");
      Serial.println("[T41]   CAL_SPAN_<1-4>     - Start span calibration");
      Serial.println("[T41]   CAL_SPAN_ADD <ch> <kg> - Add span calibration point");
      Serial.println("[T41]   CAL_SPAN_COMPUTE <ch>  - Compute span coefficients");
      Serial.println("[T41]   CAL_SHUNT_<1-4>    - Perform shunt calibration");
      Serial.println("[T41] ");
      Serial.println("[T41] Step C - Matrix Calibration:");
      Serial.println("[T41]   CAL_MATRIX_START   - Start matrix calibration");
      Serial.println("[T41]   CAL_MATRIX_ADD <kg> <x> <y> - Add matrix point");
      Serial.println("[T41]   CAL_MATRIX_COMPUTE - Compute matrix coefficients");
      Serial.println("[T41] ");
      Serial.println("[T41] Calibrated Measurements:");
      Serial.println("[T41]   CAL_FORCE_DATA     - Show calibrated force/COP data (int16)");
      Serial.println("[T41]   CAL_FORCE_INT16    - Show force data as int16 only (10g precision)");
      Serial.println("[T41] ");
      Serial.println("[T41] Validation & Testing:");
      Serial.println("[T41]   VAL_START          - Start validation system");
      Serial.println("[T41]   VAL_ADD_TEST <name> <kg> <x> <y> - Add validation test");
      Serial.println("[T41]   VAL_RUN_TESTS      - Run all validation tests");
      Serial.println("[T41]   VAL_SHOW_RESULTS   - Show validation results");
      Serial.println("[T41]   VAL_REPEATABILITY <kg> <x> <y> <n> - Repeatability test");
      Serial.println("[T41] ");
      Serial.println("[T41] Diagnostics:");
      Serial.println("[T41]   DIAG_LC            - Load cell diagnostics");
      Serial.println("[T41]   DIAG_ADC           - ADC diagnostics");
      Serial.println("[T41] ");
      Serial.println("[T41] 🤖 Automated Calibration:");
      Serial.println("[T41]   AUTO_CAL_START     - Start automated calibration");
      Serial.println("[T41]   AUTOMATED_CALIBRATION - Start automated calibration");
      Serial.println("[T41]   CONTINUE           - Continue to next step");
      Serial.println("[T41]   SKIP               - Skip current step");
      Serial.println("[T41]   ABORT              - Abort automated calibration");
      Serial.println("[T41]   STATUS             - Show calibration progress");
      Serial.println("[T41] ");
      Serial.println("[T41] 🔧 Noise Filtering:");
      Serial.println("[T41]   FILTER_ENABLE      - Enable noise filtering");
      Serial.println("[T41]   FILTER_DISABLE     - Disable noise filtering");
      Serial.println("[T41]   FILTER_STATUS      - Show filter status");
      Serial.println("[T41]   FILTER_RESET       - Reset all filters");
      Serial.println("[T41]   FILTER_REALTIME    - Real-time preset (EMA + Adaptive)");
      Serial.println("[T41]   FILTER_HIGH_QUALITY - High-quality preset (Combined)");
      Serial.println("[T41]   FILTER_LOW_NOISE   - Low-noise preset (Median + IQR)");
      Serial.println("[T41]   FILTER_GAUSSIAN    - Gaussian filter preset");
      Serial.println("[T41]   FILTER_TYPE <0-7>  - Set filter type");
      Serial.println("[T41]   OUTLIER_METHOD <0-4> - Set outlier detection");
      Serial.println("[T41]   GAUSSIAN_SIGMA <0.1-5.0> - Set Gaussian filter sigma");
      Serial.println("[T41]   SHOW_FILTERED      - Show raw vs filtered readings");
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
  
  // Initialize calibration system
  Serial.println("[T41] Initializing calibration system...");
  if (load_calibration_from_eeprom()) {
    Serial.println("[T41] ✓ Calibration data loaded from EEPROM");
    restore_adc_calibration();
  } else {
    Serial.println("[T41] ⚠️ No valid calibration data found - system needs calibration");
  }
  
  // Initialize noise filtering system
  Serial.println("[T41] Initializing noise filtering system...");
  set_realtime_filtering();  // Start with real-time preset
  enable_filtering(true);    // Enable filtering by default
  Serial.println("[T41] ✓ Noise filtering enabled (Real-time preset)");
  
  Serial.println("[T41] Ready - waiting for commands");
  Serial.println("[T41] Type 'HELP' for available commands");
  Serial.println("[T41] Quick start: ZERO (to zero), START (to begin)");
  Serial.println("[T41] Advanced: CAL_SUMMARY (show calibration status)");
  Serial.println("[T41] ==========================================");
  
  // Send initial state to ESP32
  send_state_update();
}

// ============================================================================
// FORCE DATA CONVERSION FUNCTIONS
// ============================================================================

// Convert force in Newtons to int16 with 10g precision (decagram)
// Example: 22.34 N (2.278 kg) -> 2278 (representing 227.8 * 10g units)
// Max: 200kg -> 20000 units (1 unit = 10g)
int16_t force_to_int16_decigrams(double force_newtons) {
  // Convert N to grams: 1N ≈ 101.97g (1N = 1kg * 9.81m/s² / 9.81m/s² * 1000g/kg / 9.81)
  // Actually: 1N = 1kg-force / 9.81 * 1000g = 101.97g
  // But for simplicity: 1N ≈ 102g
  double force_grams = force_newtons * 101.97;  // Convert N to grams
  int16_t result = (int16_t)round(force_grams / 10.0);  // Convert to 10g units (1 unit = 10g)
  return result;
}

// Convert moment in N⋅m to int16 with appropriate precision
// Scale moments to fit int16 range appropriately
int16_t moment_to_int16_scaled(double moment_nm) {
  // Scale moment to fit reasonable range
  // Typical moments are small, so multiply by 1000 for precision
  int16_t result = (int16_t)round(moment_nm * 1000.0);
  return result;
}

// Convert COP in mm to int16 (already in mm, just round to integer)
int16_t cop_to_int16_mm(double cop_mm) {
  return (int16_t)round(cop_mm);
}

void loop() {
  // Handle ESP32 commands first
  handle_esp32_commands();
  
  // Handle Serial Monitor commands for testing
  handle_serial_monitor_commands();
  
  // Update automated calibration system
  update_automated_calibration();
  
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
      
      // Apply offset if enabled
      int32_t processed_value = apply_offset(filtered_value, current_channel);
      
      // Store in appropriate buffer
      sample_buffer[current_channel][buffer_index] = processed_value;
      
      // Collect calibration samples if in calibration mode
      if (calibration_mode && calibration_sample_count < 100) {
        calibration_samples[current_channel][calibration_sample_count] = filtered_value;
        
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
